/**
 *******************************************************************************
 * @file      :chassis.cpp
 * @brief     : 底盘运动模块实现（工作模式 + 麦轮逆解 + 轮速闭环）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 完成编写
 *  V1.1.0      2026-11-18      Chassis        2. 轮位顺序按实车表、电流输入、四轮电流合计限幅
 *******************************************************************************
 * @attention :
 *  1. 坐标系与符号（务必与云台板约定一致）：
 *       δ = 云台朝向 − 车头朝向（云台相对底盘的偏航角，逆时针为正）
 *       平移速度 (vx, vy) 定义在云台系；先旋转到车体系，再做逆解：
 *         vx_C =  vx_G * cosδ − vy_G * sinδ
 *         vy_C =  vx_G * sinδ + vy_G * cosδ
 *       即 v_C = R(δ) * v_G（R 为绕 +z 轴逆时针旋转矩阵，(x 前, y 左, z 上) 右手系）。
 *       例：δ = +90°（枪口指向车体左侧），摇杆推「前」→ 车体向**左**平走 = 枪口方向。
 *       考核说明 3.2 给出的公式是 v_C = R(−δ) * v_G，等价于把 δ 定义成「车头 − 云台」，
 *       两者只是 δ 的符号约定不同。若实车验证方向相反，改 kDeltaSign 一处即可。
 *  2. 大角速度 / 大速度 / 大电流都必须限幅（安全红线：小陀螺必须限速）。
 *  3. 四轮输出在写进电机之前先做「电流绝对值合计」等比例限幅，防止堵转时
 *     四轮同时拉满把电池 / 分电板顶到保护。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "chassis.hpp"

#include <cmath>

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

namespace {

using hello_world::chassis_ik_solver::kWheelNum;

/* δ 的符号约定：若云台板发来的 yaw 正方向与底盘定义相反，把它改成 -1.0f */
/* δ 的符号约定（实车标定项）
 *   1  = 云台关节角逆时针为正（云台 yaw 电机正方向 = 车体俯视逆时针）
 *  -1  = 相反
 *
 * 【2026-11-18 标定结论：+1】
 *   曾经因为「跟随模式一直转」误判成符号问题改成 -1，后来定位到真因是：
 *     云台在保护态时不刷新 actual_，G2C 上报的是**冻结的**关节角。
 *     跟随环因此看到恒定误差 → 匀速自转（不是符号反）。
 *   该 bug 已修（云台 Robot::update 的急停分支也调 gimbal_->update）。
 *   实测判据：δ 用 -1 时「分离模式推前进，车体左右镜像」——只有 R(+δ) 才对，
 *   说明云台上报的关节角就是逆时针为正，（与 R(+δ) 的矩阵约定相符）。
 *   ⚠ 若将来更换云台 yaw 电机 / 重装方向，先用「云台转 90° 左、推前进应向左走」
 *     复核这一项。 */
constexpr float kDeltaSign = 1.0f;

/* 单轮轮速目标下发给速度环时的上下限（与 Chassis::kMaxWheelVel 一致） */
constexpr float kWheelVelLimit = robot::Chassis::kMaxWheelVel;

}  // namespace

/* Exported function definitions ---------------------------------------------*/

namespace robot {

Chassis::Chassis(void)
{
  /* 跟随模式角速度环：误差取 δ，控制律 w = Kp * δ 使 δ → 0 */
  follow_pid_.init(kFollowKp, kFollowKi, kFollowKd, 0.0f, kFollowOutLim);
  reset();
}

void Chassis::registerMotor(int idx, hello_world::motor::Motor *motor)
{
  if (idx < 0 || idx >= kWheelNum) {
    return;
  }
  motors_[idx] = motor;
}

void Chassis::registerPid(int idx, hello_world::pid::Pid *pid)
{
  if (idx < 0 || idx >= kWheelNum) {
    return;
  }
  pids_[idx] = pid;
}

void Chassis::registerIkSolver(
    hello_world::chassis_ik_solver::ChassisIkSolver *solver)
{
  ik_solver_ = solver;
}

bool Chassis::isReady(void) const
{
  if (ik_solver_ == nullptr) {
    return false;
  }
  for (int i = 0; i < kWheelNum; ++i) {
    if (motors_[i] == nullptr || pids_[i] == nullptr) {
      return false;
    }
  }
  return true;
}

void Chassis::update(ChassisMode mode, float delta, float dt)
{
  /* 模块没注册完（或注册失败）：直接归零，绝不带着半边模块去驱动电机 */
  if (!isReady()) {
    reset();
    return;
  }

  /* 0. δ 归一化到 [-π, π)，避免 359° → 1° 被当成反向大转（考核说明 3.5） */
  delta_ = kDeltaSign * NormalizeAngleRad(delta);
  dt_ = (dt > 0.0f) ? dt : kDefaultDt;

  /* 1. 按工作模式解算速度指令（云台系平移 + 角速度） */
  ChassisSpeed raw;
  solveMode(mode, &raw);

  /* 2. 速度限幅（不同模式的上限不同） */
  raw.vx = ClampValue(raw.vx, -kMaxVelX, kMaxVelX);
  raw.vy = ClampValue(raw.vy, -kMaxVelY, kMaxVelY);
  if (mode == ChassisMode::kSpin) {
    raw.w = ClampValue(raw.w, -kMaxSpinOmega, kMaxSpinOmega);
  } else if (mode == ChassisMode::kFollow) {
    raw.w = ClampValue(raw.w, -kFollowOutLim, kFollowOutLim);
  } else {
    raw.w = ClampValue(raw.w, -kMaxOmegaManual, kMaxOmegaManual);
  }

  /* 3. 加速度限幅：模式切换 / 摇杆突变时不会出现电机突跳 */
  const ChassisSpeed cmd = slewLimit(raw, dt_);

  /* 4. 云台系 → 车体系（前进正方向始终是云台朝向） */
  float vx_chassis = 0.0f;
  float vy_chassis = 0.0f;
  rotateGimbalToChassis(cmd.vx, cmd.vy, delta_, &vx_chassis, &vy_chassis);
  chassis_speed_.vx = vx_chassis;
  chassis_speed_.vy = vy_chassis;
  chassis_speed_.w = cmd.w;

  /* 5. 麦克纳姆轮逆解（45° 滚子角） */
  float wheel_vel[kWheelNum] = {0.0f, 0.0f, 0.0f, 0.0f};
  ik_solver_->solve(vx_chassis, vy_chassis, cmd.w, wheel_vel);

  /* 6. 逆解结果限幅：按最大轮速等比缩放，保证方向不变（只降速，不改变运动方向） */
  float max_abs = 0.0f;
  for (int i = 0; i < kWheelNum; ++i) {
    const float abs_v = fabsf(wheel_vel[i]);
    if (abs_v > max_abs) {
      max_abs = abs_v;
    }
  }
  if (max_abs > kWheelVelLimit) {
    const float scale = kWheelVelLimit / max_abs;
    for (int i = 0; i < kWheelNum; ++i) {
      wheel_vel[i] *= scale;
    }
  }

  /* 7. 逐轮速度环闭环（输出单位：A） */
  for (int i = 0; i < kWheelNum; ++i) {
    wheel_target_vel_[i] = wheel_vel[i];
    wheel_actual_vel_[i] = motors_[i]->vel();

    const float err = wheel_target_vel_[i] - wheel_actual_vel_[i];
    wheel_output_[i] = pids_[i]->calc(err, wheel_actual_vel_[i], dt_);
  }

  /* 8. 四轮电流合计限幅（等比例缩放，保持四轮配比与方向） */
  limitCurrentSum();

  /* 9. 下发到电机（Motor::setInput 内部还会按单轮 20A 再限一次） */
  for (int i = 0; i < kWheelNum; ++i) {
    motors_[i]->setInput(wheel_output_[i]);
  }
}

void Chassis::limitCurrentSum(void)
{
  /* 四轮电流绝对值合计限幅：等比例缩放，保持四轮配比与方向。
   * 不做这一步的话，堵转 / 阶跃时四轮同时拉满可能瞬时 40A+，电池/分电板直接保护。 */
  float sum_abs = 0.0f;
  for (int i = 0; i < kWheelNum; ++i) {
    sum_abs += fabsf(wheel_output_[i]);
  }
  if (sum_abs > kWheelCurrentSumLimit) {
    const float scale = kWheelCurrentSumLimit / sum_abs;
    for (int i = 0; i < kWheelNum; ++i) {
      wheel_output_[i] *= scale;
    }
  }
}

void Chassis::reset(void)
{
  input_.reset();
  cmd_smoothed_.reset();
  chassis_speed_.reset();
  delta_ = 0.0f;

  follow_pid_.reset();

  for (int i = 0; i < kWheelNum; ++i) {
    if (pids_[i] != nullptr) {
      pids_[i]->reset();
    }
    wheel_target_vel_[i] = 0.0f;
    wheel_actual_vel_[i] = 0.0f;
    wheel_output_[i] = 0.0f;
    /* 失效归零：必须让电机立刻收到 0（硬性验收项） */
    if (motors_[i] != nullptr) {
      motors_[i]->setInput(0.0f);
    }
  }
}

/* Private function definitions ----------------------------------------------*/
void Chassis::solveMode(ChassisMode mode, ChassisSpeed *raw)
{
  raw->reset();

  /* 平移速度在所有模式下都来自摇杆，且都定义在云台系 */
  raw->vx = input_.vx;
  raw->vy = input_.vy;

  switch (mode) {
    case ChassisMode::kFollow: {
      /* 跟随：w = Kp * δ（δ → 0 即车头对准云台）；死区内不动作，避免原地抖动。
       *
       * 关键安全前提：云台必须处于「绝对朝向」模式（kCartesian）。
       * 若云台是关节空间控制，底盘一转云台就跟着转、δ 永远不变，跟随环会输出
       * 一个恒定角速度 → 底盘匀速自转停不下来（这就是考核说明里的「电机失控
       * 持续转动」）。所以这里宁可不动：follow_rotation_allowed 为 false 时 w = 0，
       * 只保留云台系平移。 */
      if (!input_.follow_rotation_allowed) {
        follow_pid_.reset();
        raw->w = 0.0f;
        break;
      }
      if (fabsf(delta_) <= kFollowDeadBand) {
        follow_pid_.reset();
        raw->w = 0.0f;
      } else {
        /* PD：比例看角度误差，阻尼看底盘自身角速度。
       * 只给 P 时，轮速环 ~0.1s 滞后 + G2C 10ms 传输延迟会让环路相位裕度不足，
       * 形成极限环 —— 实测表现为车在顺时针/逆时针之间来回震荡。 */
      raw->w = (kFollowKp * delta_) - (kFollowKd * input_.yaw_rate);
      }
      break;
    }
    case ChassisMode::kSpin: {
      /* 小陀螺：基准自转 + 拨轮调制（限幅在 update 里做）。
       * 拨轮回中位时仍有基准自转，符合 3.1 的「绕自身几何中心持续自旋」；
       * 拨轮推到负向可以把转速压到 0 —— 想停转就出档位，而不是靠拨轮。 */
      raw->w = kSpinBaseRate + input_.w_spin;
      break;
    }
    case ChassisMode::kSeparate:
    default: {
      /* 分离：w 直接来自拨轮，底盘不自动回正、不自旋 */
      raw->w = input_.w_manual;
      break;
    }
  }
}

ChassisSpeed Chassis::slewLimit(const ChassisSpeed &target, float dt)
{
  if (!(dt > 0.0f)) {
    dt = kDefaultDt;
  }

  const float max_dvx = kMaxAccel * dt;
  const float max_dvy = kMaxAccel * dt;
  const float max_dw = kMaxAngAccel * dt;

  cmd_smoothed_.vx += ClampValue(target.vx - cmd_smoothed_.vx, -max_dvx, max_dvx);
  cmd_smoothed_.vy += ClampValue(target.vy - cmd_smoothed_.vy, -max_dvy, max_dvy);
  cmd_smoothed_.w += ClampValue(target.w - cmd_smoothed_.w, -max_dw, max_dw);

  return cmd_smoothed_;
}

void Chassis::rotateGimbalToChassis(float vx_gimbal, float vy_gimbal, float delta,
                                    float *vx_chassis, float *vy_chassis)
{
  const float cos_d = cosf(delta);
  const float sin_d = sinf(delta);

  if (vx_chassis != nullptr) {
    *vx_chassis = vx_gimbal * cos_d - vy_gimbal * sin_d;
  }
  if (vy_chassis != nullptr) {
    *vy_chassis = vx_gimbal * sin_d + vy_gimbal * cos_d;
  }
}

}  // namespace robot
