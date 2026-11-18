/**
 *******************************************************************************
 * @file      :robot.cpp
 * @brief     : 底盘板顶层状态机实现
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 完成编写
 *******************************************************************************
 * @attention :
 *  本文件的 update() 是底盘板每一个 1ms 周期的全流程：
 *      updateData()         读数据（IMU 采样、遥控器在线、双板在线、δ）
 *      updateSafety()       安全判定（急停 / 离线 / 保护）——放在最外层
 *      updateStateMachine() Dead → Resurrection → Working 三态迁移
 *      generateCommand()    仅在允许输出时生成指令
 *      zeroOutput()         其余情况一律立刻归零
 *  顺序不可调换，「先安全、后控制」是硬性要求（考核说明 1.3、4.9）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "robot.hpp"

#include <cmath>

#include "chassis.hpp"
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"
#include "ins_comm.hpp"
#include "ins_motor.hpp"
#include "tick.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

namespace {

/* 摇杆 / 拨轮死区处理：死区内输出 0，死区外线性映射到 [-1, 1]，
 * 这样既不会因为摇杆中位漂移而「自己动」，又不会在刚出死区时出现跳变 */
float ApplyDeadzone(float value, float deadzone)
{
  /* 越界保护：摇杆 / 拨轮的正常范围是 [-1, 1]。超出说明通道异常 ——
   * 实测过一台遥控器的拨轮电位器一端悬空，DBUS 里读到 0x1E25（≈10.1），
   * 如果不拦，映射出来是 11.5，再限幅就是**满速指令**，车会一直满速自转。
   * 这里直接当 0 处理（失效即零输出），比限幅到 ±1 安全。
   * 写成 !(a) || !(b) 是为了同时把 NaN 挡掉。 */
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

}  // namespace

/* Exported function definitions ---------------------------------------------*/

namespace robot {

void Robot::update(void)
{
  /* 1. 更新数据 */
  updateData();

  /* 2. 安全判定（最外层，任何控制分支之前） */
  updateSafety();

  /* 3. 状态机迁移；急停 / 离线时直接回 Dead，不允许在保护状态下「继续跑逻辑」 */
  if (is_emergency_) {
    enterDead();
  } else {
    updateStateMachine();
  }

  /* 4. 生成并下发指令：不满足输出条件时一律立刻归零 */
  const bool can_output =
      isOutputAllowed() && (chassis_ != nullptr) && (rc_ != nullptr);
  if (can_output) {
    generateCommand();
  } else {
    zeroOutput();
  }
  is_output_enabled_ = can_output;

  /* 5. 把本板状态写进 C2G 帧（CommTask 下一拍发出去） */
  updateCommData();
}

/* Private function definitions ----------------------------------------------*/
void Robot::updateData(void)
{
  /* IMU：采样 + 零漂估计 + 姿态解算（1ms 一次） */
  if (imu_ != nullptr) {
    imu_->update();
    is_imu_ready_ = imu_->isReady();
  } else {
    is_imu_ready_ = false;
  }

  /* 四轮编码器反馈：任何一个掉线都不许出力。
   * 轮速环拿不到反馈时，误差恒等于目标，P/I 会把电流推到限幅，
   * 架空的车轮就会"越转越快"停不下来。这条判据是那类失控的兜底。 */
  is_wheel_feedback_ok_ = true;
  for (int i = 0; i < 4; ++i) {
    if (!IsWheelFeedbackOnline(i)) {
      is_wheel_feedback_ok_ = false;
      break;
    }
  }

  /* 遥控器：在线状态来自 Instance 层的掉线检测（UART 回调里刷新） */
  is_rc_online_ = IsRemoteControlOnline();

  if (rc_ != nullptr) {
    /* 右拨杆：上 = 运行；中 / 下 = 都不允许出力（下为急停） */
    const uint8_t right_switch = static_cast<uint8_t>(rc_->rc_r_switch());
    is_manual_enabled_ = (right_switch == kRightSwitchRun);

    /* 左拨杆 = 底盘工作模式：上分离 / 中跟随 / 下小陀螺。
     * 必须先去抖：拨杆接触不良或个别 DBUS 帧误码会让档位瞬变，一旦瞬变到小陀螺，
     * w 会被加上 1.2 rad/s 的基准自转（且恒为正），表现就是「偶发的转向全反」。 */
    const uint8_t left_switch = static_cast<uint8_t>(rc_->rc_l_switch());
    if (left_switch != mode_candidate_) {
      mode_candidate_ = left_switch;
      mode_candidate_cnt_ = 0;
    } else if (mode_candidate_cnt_ < kModeDebounceCycles) {
      ++mode_candidate_cnt_;
    }

    /* 只有连续 kModeDebounceCycles 个周期保持不变，才真正切换模式 */
    if (mode_candidate_cnt_ >= kModeDebounceCycles) {
      switch (mode_candidate_) {
        case kLeftSwitchFollow:
          chassis_mode_ = ChassisMode::kFollow;
          break;
        case kLeftSwitchSpin:
          chassis_mode_ = ChassisMode::kSpin;
          break;
        case kLeftSwitchSeparate:
        default:
          chassis_mode_ = ChassisMode::kSeparate;
          break;
      }
    }
  } else {
    is_manual_enabled_ = false;
    chassis_mode_ = ChassisMode::kSeparate;
    mode_candidate_ = 0;
    mode_candidate_cnt_ = 0;
  }

  /* 双板通讯：判离线 + 取 δ（云台 yaw 关节角）、pitch 与云台模式 */
  if (comm_ != nullptr) {
    is_gimbal_online_ = !comm_->isOffline();
    if (is_gimbal_online_) {
      /* δ 只有在云台自报 Working 时才是可信的关节角：Dead / Resurrection 期间
       * 云台报的角没有意义（保护态下它甚至不刷新 actual_，会一直发进入保护前的旧值）。 */
      is_gimbal_ready_ =
          (comm_->gimbal_data().life_state == LifeState::kWorking);
      is_gimbal_absolute_ = is_gimbal_ready_ &&
                            (comm_->gimbal_data().mode == GimbalMode::kCartesian);
      if (is_gimbal_ready_) {
        /* δ 归一化到 [-π, π)，避免 359° → 1° 被当成反向大转 */
        delta_ = NormalizeAngleRad(comm_->gimbal_data().yaw);
        gimbal_pitch_ = comm_->gimbal_data().pitch;
      } else {
        /* 云台不可信 → δ 当 0，平移退化成底盘坐标系。
         * ⚠ 这里以前无条件取 δ，结果云台在保护态时上报的旧角（实测出现过 142°）
         *   被拿去旋转平移，表现为「分离模式推前进，车却朝车尾走」。 */
        delta_ = 0.0f;
        gimbal_pitch_ = 0.0f;
      }
    } else {
      delta_ = 0.0f;
      gimbal_pitch_ = 0.0f;
      is_gimbal_ready_ = false;
      is_gimbal_absolute_ = false;
    }
  } else {
    is_gimbal_online_ = false;
    delta_ = 0.0f;
    gimbal_pitch_ = 0.0f;
    is_gimbal_ready_ = false;
    is_gimbal_absolute_ = false;
  }
}

void Robot::updateSafety(void)
{
  const bool is_emergency_switch =
      (rc_ != nullptr) &&
      (static_cast<uint8_t>(rc_->rc_r_switch()) == kRightSwitchEmergency);

  /* 三条硬性归零条件（考核说明 1.3）：
   *   1) 急停档位
   *   2) 遥控器离线 / 数据长时间未更新
   *   3) 双板通讯离线（双板互保：任一板离线两板都进保护）                */
  is_emergency_ = (!is_rc_online_) || is_emergency_switch ||
                  (kRequireGimbalOnline && !is_gimbal_online_);
}

void Robot::updateStateMachine(void)
{
  switch (life_state_) {
    case LifeState::kDead: {
      /* Dead：输出全零，等待就绪条件（遥控器在线 + IMU 零漂收敛 + 双板在线） */
      if (isWorkingConditionMet()) {
        life_state_ = LifeState::kResurrection;
        resurrection_tick_ = hello_world::tick::GetTickMs();
        /* 进入自检前先清掉上一状态残留的指令与积分 */
        resetControllers();
      }
      break;
    }
    case LifeState::kResurrection: {
      /* Resurrection：保持零输出，完成自检复位并稳定一段时间 */
      if (!isWorkingConditionMet()) {
        enterDead();
        break;
      }
      zeroOutput();
      if ((hello_world::tick::GetTickMs() - resurrection_tick_) >=
          kResurrectionHoldMs) {
        life_state_ = LifeState::kWorking;
        /* 切到 Working 前再清一次，确保第一拍不会带着旧积分突变 */
        resetControllers();
      }
      break;
    }
    case LifeState::kWorking: {
      /* Working：正常控制；条件不满足立刻回 Dead 并归零 */
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

void Robot::generateCommand(void)
{
  if (chassis_ == nullptr || rc_ == nullptr) {
    zeroOutput();
    return;
  }

  /* 摇杆 → 云台系平移速度（左摇杆：上 = 前进，左 = 左移）
   *
   * ⚠ kStickVxSign：左摇杆竖直轴（vx）的符号标定。
   *    1  = 上推为正（DT7 的常见约定）
   *   -1  = 上推为负（**2026-11-18 实车实测：这台车的遥控器是这种**）
   *   定标依据：跟随方向正确（说明 δ 没错、R(δ) 没错）、左右正确（说明 vy 没错），
   *   只有"前进/后退"整体反向 ⇒ 只可能是 vx 这一路的符号，与坐标系变换无关
   *   （R(π) 会同时把左右也反过来，那就不符合观测了）。
   *   若以后换遥控器，把这里改回 1.0f 即可。 */
  constexpr float kStickVxSign = -1.0f;
  /* kStickVySign：左摇杆水平轴（vy）的符号标定。
   * 这里写成 vy = -stick_h（"推右 → 云台系 +y 的相反方向"，即车右移）。
   * 2026-11-18 实车：翻正 vx 之后"前进对了、左右镜像" ⇒ 这台车的水平轴同样与代码假设
   * 相反，这里取 +1 抵消（等价于 vy 与推杆同号）。
   * 注意 w（拨轮/跟随）是独立的第三路，实测跟随方向正确 ⇒ 不要动。
   * 若以后换遥控器，vx/vy 两个符号一起回归 +1 / -1 即可。 */
  constexpr float kStickVySign = 1.0f;
  const float vx = kStickVxSign * ApplyDeadzone(rc_->rc_lv(), kStickDeadzone) *
                   Chassis::kMaxVelX;
  const float vy = kStickVySign * ApplyDeadzone(rc_->rc_lh(), kStickDeadzone) *
                   Chassis::kMaxVelY;

  /* 拨轮 → 底盘转向通道：
   *   分离模式：直接映射成角速度（不自动回正、不自旋）
   *   小陀螺  ：映射成自旋调制量，叠加到 Chassis::kSpinBaseRate 上并限幅 */
  const float wheel = ApplyDeadzone(rc_->rc_wheel(), kStickDeadzone);
  const float wheel_spin = ApplyDeadzone(rc_->rc_wheel(), kSpinDeadzone);

  ChassisInput input;
  input.vx = vx;
  input.vy = vy;
  input.w_manual = wheel * Chassis::kMaxOmegaManual;
  input.w_spin = wheel_spin * Chassis::kSpinWheelGain;
  /* 跟随环的安全前提：云台必须是绝对朝向模式，否则 δ 不随底盘转动而变，
   * 跟随会退化成匀速自转（见 kFollowRequireAbsoluteGimbal 的说明） */
  input.follow_rotation_allowed =
      (!kFollowRequireAbsoluteGimbal) || is_gimbal_absolute_;

  /* 底盘自身角速度（跟随环阻尼项的反馈量）
   * 用 IMU yaw 逐拍差分 + 一阶低通（时间常数约 20 个周期 = 20ms）：
   *   - 直接差分噪声大，会让阻尼项发抖；
   *   - 低通太重会引入相位滞后，抵消阻尼效果。
   * IMU 未就绪时给 0，跟随退化为纯 P（由 follow_rotation_allowed 兜底）。 */
  float yaw_rate = 0.0f;
  if ((imu_ != nullptr) && is_imu_ready_) {
    const float imu_yaw = imu_->yaw();
    if (has_last_imu_yaw_) {
      const float raw_rate =
          NormalizeAngleRad(imu_yaw - last_imu_yaw_) / kLoopDt;
      chassis_yaw_rate_ += 0.05f * (raw_rate - chassis_yaw_rate_);
      yaw_rate = chassis_yaw_rate_;
    }
    last_imu_yaw_ = imu_yaw;
    has_last_imu_yaw_ = true;
  } else {
    has_last_imu_yaw_ = false;
    chassis_yaw_rate_ = 0.0f;
  }
  input.yaw_rate = yaw_rate;

  chassis_->setInput(input);
  chassis_->update(chassis_mode_, delta_, kLoopDt);
}

void Robot::zeroOutput(void)
{
  /* 所有电机立刻 setInput(0)：由 Chassis::reset() 完成（含 PID 复位） */
  if (chassis_ != nullptr) {
    chassis_->reset();
  }
}

void Robot::resetControllers(void)
{
  zeroOutput();
}

void Robot::enterDead(void)
{
  life_state_ = LifeState::kDead;
  resurrection_tick_ = 0;
  zeroOutput();
}

void Robot::updateCommData(void)
{
  if (comm_ == nullptr) {
    return;
  }

  GimbalChassisComm::ChassisData &data = comm_->chassis_data();
  data.is_ready = (life_state_ == LifeState::kWorking);
  data.is_emergency = is_emergency_;
  data.life_state = life_state_;
  data.mode = chassis_mode_;
  /* 转发右摇杆：云台板没有本地 DBUS 时靠它瞄准（接收机只接底盘板的接法） */
  data.aim_yaw = (rc_ != nullptr) ? rc_->rc_rh() : 0.0f;
  data.aim_pitch = (rc_ != nullptr) ? rc_->rc_rv() : 0.0f;

  /* 底盘期望的云台相对角：跟随模式下希望 δ → 0（车头对准云台）；
   * 其余模式不要求云台转动，直接下发当前 δ（云台板可选择忽略这两个字段） */
  if (chassis_mode_ == ChassisMode::kFollow) {
    data.target_gimbal_yaw = 0.0f;
  } else {
    data.target_gimbal_yaw = delta_;
  }
  data.target_gimbal_pitch = gimbal_pitch_;
}

bool Robot::isOutputAllowed(void) const
{
  /* 输出必须同时满足：进 Working、无急停/离线、右拨杆在运行档、四轮反馈都还在。
   * 最后一条是「空载失控」的兜底：轮速环一旦拿不到反馈，积分会把电流推到限幅，
   * 架空的车轮会一路加速停不下来；反馈一断就直接不许出力。 */
  return (life_state_ == LifeState::kWorking) && (!is_emergency_) &&
         is_manual_enabled_ && is_wheel_feedback_ok_;
}

bool Robot::isWorkingConditionMet(void) const
{
  if (!is_rc_online_) {
    return false;
  }
  if (kRequireImuReady && !is_imu_ready_) {
    return false;
  }
  if (kRequireGimbalOnline && !is_gimbal_online_) {
    return false;
  }
  return true;
}

}  // namespace robot
