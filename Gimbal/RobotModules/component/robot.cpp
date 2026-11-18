/**
 *******************************************************************************
 * @file      :robot.cpp
 * @brief     : 云台板顶层状态机实现
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 完成编写
 *******************************************************************************
 * @attention :
 *  顺序不可调换：先安全、后控制（考核说明 1.3、4.9）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "robot.hpp"

#include <cmath>

#include "gimbal.hpp"
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"
#include "ins_comm.hpp"

namespace robot {

namespace {

/* 摇杆死区处理：死区内输出 0，死区外线性映射到 [-1, 1]。
 * 越界值（|x| > 1.05）说明通道异常，直接当 0 —— 失效即零输出，比限幅到 ±1 安全。 */
float ApplyDeadband(float value, float deadzone)
{
  constexpr float kMaxValidStick = 1.05f;
  if (!(value >= -kMaxValidStick) || !(value <= kMaxValidStick)) {
    return 0.0f;
  }
  if (deadzone <= 0.0f || deadzone >= 1.0f) {
    return value;
  }
  if (value > deadzone) {
    return (value - deadzone) / (1.0f - deadzone);
  }
  if (value < -deadzone) {
    return (value + deadzone) / (1.0f - deadzone);
  }
  return 0.0f;
}

/* 摇杆回中判定用的死区（比控制死区略大一点，避免中位抖动反复清计数） */
constexpr float kCenterDeadzone = 0.02f;
/* 连续回中多少个周期后才允许积分目标角（1kHz 下 50 = 50ms） */
constexpr uint32_t kArmCycles = 50u;

}  // namespace

void Robot::init(const Config &cfg)
{
  cfg_ = cfg;
  life_state_ = LifeState::kDead;
  state_cycles_ = 0;

  is_emergency_ = false;
  is_rc_online_ = false;
  is_manual_enabled_ = false;
  is_imu_ready_ = false;

  input_armed_ = false;
  centered_cycles_ = 0;
  target_ = GimbalAngle{};

  has_last_cont_yaw_ = false;
  last_cont_yaw_ = 0.0f;
  yaw_rate_ = 0.0f;

  if (gimbal_ != nullptr) {
    gimbal_->stop();
  }
}

void Robot::registerGimbal(Gimbal *gimbal) { gimbal_ = gimbal; }

GimbalMode Robot::mode(void) const
{
  /* 上报「实际生效」的模式：绝对角配了 true 但 IMU 未就绪时，Gimbal 内部会退回
   * 关节角控制；此时若仍报 kCartesian，底盘会拿关节角当世界角去追，表现为匀速
   * 自转停不下来。 */
  if (gimbal_ != nullptr) {
    return gimbal_->absoluteActive() ? GimbalMode::kCartesian : GimbalMode::kJoint;
  }
  return cfg_.absolute_yaw ? GimbalMode::kCartesian : GimbalMode::kJoint;
}

void Robot::update(float dt)
{
  if (!(dt > 0.0f)) {
    return;
  }

  /* 1. 更新数据（IMU 采样、遥控在线、档位） */
  updateData();

  /* 2. 安全判定（最外层） */
  updateSafety();

  /* 3. 急停 / 离线：立刻卸力并回 Dead，不允许在保护状态下继续跑逻辑 */
  if (is_emergency_) {
    input_armed_ = false;
    centered_cycles_ = 0;
    enterDead();
    /* 卸力之后仍要刷新 actual_：否则 G2C 会一直上报"进入保护前"的旧关节角，
     * 底盘拿它去旋转平移就会走偏（实测云台摆正车头时上报了 142°）。
     * update() 在未使能态只读角度、不发力，安全。 */
    if (gimbal_ != nullptr) {
      gimbal_->update(dt);
    }
    updateCommData();
    return;
  }

  /* 4. 状态机迁移 */
  updateStateMachine();

  /* 5. 生成目标角并下发（不允许出力时一律 setInput(0)） */
  if (isOutputAllowed()) {
    generateCommand(dt);
  } else {
    zeroOutput();
    /* 卸力之后仍要刷新实际角：否则 stop() 里「目标跟随实际」会停在旧值 */
    if (gimbal_ != nullptr) {
      gimbal_->update(dt);
    }
  }

  /* 6. 把本板状态写进 G2C 帧（CommTask 下一拍发出去） */
  updateCommData();
}

void Robot::updateData(void)
{
  /* IMU：采样 + 零漂估计 + 姿态解算（1ms 一次） */
  if (imu_ != nullptr) {
    imu_->update();
    is_imu_ready_ = imu_->isReady();
  } else {
    is_imu_ready_ = false;
  }

  /* 遥控器：在线状态来自 Instance 层的掉线检测（UART 回调里刷新） */
  is_rc_online_ = IsRemoteControlOnline();

  /* 对板 C2G：有效 + 对板在 Working 才算一个可用控制源（接收机只在底盘板时用） */
  has_peer_command_ = hasPeerCommand();

  if (rc_ != nullptr) {
    is_manual_enabled_ =
        (static_cast<uint8_t>(rc_->rc_r_switch()) == kRightSwitchRun);
  } else {
    is_manual_enabled_ = false;
  }
}

void Robot::updateSafety(void)
{
  const bool is_emergency_switch =
      (rc_ != nullptr) &&
      (static_cast<uint8_t>(rc_->rc_r_switch()) == kRightSwitchEmergency);

  /* 对板转发来的急停位：底盘急停时云台也必须停（双板互保） */
  const bool is_peer_estop =
      has_peer_command_ &&
      (comm_ != nullptr) && comm_->chassis_data().is_emergency;

  /* 考核说明 1.3：急停档位、**没有任何有效控制源**都必须立刻归零。
   * 控制源 = 本板 DBUS 或对板 C2G（接收机只接底盘板时，本地 DBUS 永远没有，
   * 此时若只判本地 DBUS 会让云台永远动不了 —— 所以判据必须是 hasControlSource）。*/
  is_emergency_ = (!hasControlSource()) || is_emergency_switch || is_peer_estop;
}

void Robot::updateStateMachine(void)
{
  ++state_cycles_;

  switch (life_state_) {
    case LifeState::kDead: {
      /* Dead：输出全零，等外设稳定（dead_dwell）后再进自检 */
      zeroOutput();
      if (state_cycles_ >= cfg_.dead_dwell_cycles) {
        life_state_ = LifeState::kResurrection;
        state_cycles_ = 0;
        input_armed_ = false;
        centered_cycles_ = 0;
        zeroOutput();
      }
      break;
    }
    case LifeState::kResurrection: {
      /* Resurrection：保持零输出，等 IMU 零漂收敛 + 遥控可用 */
      zeroOutput();
      if (!isWorkingConditionMet()) {
        break;
      }
      life_state_ = LifeState::kWorking;
      state_cycles_ = 0;
      /* 进 Working 时把目标角对齐当前实际角，避免第一拍出现大误差。
       * yaw 用 yawFeedback()：绝对角模式下它是 IMU 绝对角，不是关节角。 */
      if (gimbal_ != nullptr) {
        target_.yaw = gimbal_->yawFeedback();
        target_.pitch = cfg_.pitch_home;
      }
      input_armed_ = false;
      centered_cycles_ = 0;
      ++working_entry_cnt_;  /* 诊断：反复进入说明状态在抖 */
      break;
    }
    case LifeState::kWorking: {
      /* Working：条件不满足立刻回 Dead 并卸力 */
      if (!isWorkingConditionMet()) {
        enterDead();
      }
      break;
    }
    default: {
      enterDead();
      break;
    }
  }
}

void Robot::generateCommand(float dt)
{
  if (gimbal_ == nullptr || !hasControlSource()) {
    zeroOutput();
    return;
  }

  /* 反馈源刚切换（关节角 ⇄ 绝对角，通常发生在 IMU 零漂完成那一刻）：
   * 把目标角对齐到新反馈，避免切换后出现大误差猛甩。 */
  if (gimbal_->consumeFeedbackSwitch()) {
    target_.yaw = gimbal_->yawFeedback();
  }

  /* 上电 / 恢复后先锁住当前姿态，摇杆连续回中一段时间后才允许积分，
   * 避免上电瞬间的遥控残值把云台拉走。 */
  const float aim_yaw = AimYaw();
  const float aim_pitch = AimPitch();

  if (!input_armed_) {
    target_.yaw = gimbal_->yawFeedback();
    const bool sticks_centered =
        (fabsf(ApplyDeadband(aim_yaw, kCenterDeadzone)) < 1e-6f) &&
        (fabsf(ApplyDeadband(aim_pitch, kCenterDeadzone)) < 1e-6f);
    if (sticks_centered) {
      if (centered_cycles_ < kArmCycles) {
        ++centered_cycles_;
      } else {
        input_armed_ = true;
      }
    } else {
      centered_cycles_ = 0;
    }
  }

  if (input_armed_) {
    /* 右摇杆是**角速度**控制：
     *   水平推右 → yaw 顺时针（云台逆时针为正，所以取负）
     *   竖直推上 → pitch 抬升（正向） */
    const float yaw_rate =
        -ApplyDeadband(aim_yaw, cfg_.stick_deadband) * cfg_.max_yaw_rate;
    const float pitch_rate =
        ApplyDeadband(aim_pitch, cfg_.stick_deadband) * cfg_.max_pitch_rate;
    target_.yaw += yaw_rate * dt;
    target_.pitch += pitch_rate * dt;
  }

  /* 目标角钳位：不依赖下游，避免积分跑飞后要很久才能拉回来 */
  if (target_.pitch < cfg_.pitch_min) {
    target_.pitch = cfg_.pitch_min;
  }
  if (target_.pitch > cfg_.pitch_max) {
    target_.pitch = cfg_.pitch_max;
  }

  gimbal_->setTarget(target_);
  gimbal_->update(dt);

  /* 上报用的 yaw 角速度（调试 / 底盘前馈用，可忽略） */
  const float cont = gimbal_->yawContinuous();
  if (has_last_cont_yaw_) {
    yaw_rate_ = (cont - last_cont_yaw_) / dt;
  } else {
    has_last_cont_yaw_ = true;
    yaw_rate_ = 0.0f;
  }
  last_cont_yaw_ = cont;
}

void Robot::zeroOutput(void)
{
  if (gimbal_ != nullptr) {
    gimbal_->stop();
  }
  yaw_rate_ = 0.0f;
  has_last_cont_yaw_ = false;
}

void Robot::enterDead(void)
{
  life_state_ = LifeState::kDead;
  state_cycles_ = 0;
  zeroOutput();
}

void Robot::updateCommData(void)
{
  if (comm_ == nullptr) {
    return;
  }

  GimbalChassisComm::GimbalData &data = comm_->gimbal_data();
  data.is_ready = (life_state_ == LifeState::kWorking);
  data.is_emergency = is_emergency_;
  data.life_state = life_state_;
  data.mode = mode();
  if (gimbal_ != nullptr) {
    data.yaw = NormalizeAngleRad(gimbal_->actual().yaw);
    data.pitch = gimbal_->actual().pitch;
  } else {
    data.yaw = 0.0f;
    data.pitch = 0.0f;
  }
  data.yaw_rate = yaw_rate_;
}

bool Robot::hasPeerCommand(void) const
{
  if ((comm_ == nullptr) || comm_->isOffline()) {
    return false;
  }
  const GimbalChassisComm::ChassisData &d = comm_->chassis_data();
  return d.is_ready && (d.life_state == LifeState::kWorking);
}

float Robot::AimYaw(void) const
{
  /* 本地 DBUS 优先；没有就用底盘转发来的右摇杆 */
  if (is_rc_online_ && (rc_ != nullptr)) {
    return rc_->rc_rh();
  }
  if (has_peer_command_ && (comm_ != nullptr)) {
    return comm_->chassis_data().aim_yaw;
  }
  return 0.0f;
}

float Robot::AimPitch(void) const
{
  if (is_rc_online_ && (rc_ != nullptr)) {
    return rc_->rc_rv();
  }
  if (has_peer_command_ && (comm_ != nullptr)) {
    return comm_->chassis_data().aim_pitch;
  }
  return 0.0f;
}

bool Robot::isWorkingConditionMet(void) const
{
  if (cfg_.require_remote && !hasControlSource()) {
    return false;
  }
  if (cfg_.require_imu_calib && !is_imu_ready_) {
    return false;
  }
  return true;
}

bool Robot::isOutputAllowed(void) const
{
  /* 输出必须同时满足：进 Working、无急停 / 离线、右拨杆在运行档 */
  return (life_state_ == LifeState::kWorking) && (!is_emergency_) &&
         is_manual_enabled_;
}


}  // namespace robot
