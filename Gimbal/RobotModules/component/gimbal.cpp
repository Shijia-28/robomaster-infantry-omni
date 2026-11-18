/**
 *******************************************************************************
 * @file      :gimbal.cpp
 * @brief     : 云台关节空间控制实现
 * @history   :
 *  Version     Date            Author          Note
 *  V2.0.0      2026-11-18      Chassis        1. 重构内部结构（行为与 V1 完全等价）
 *******************************************************************************
 * @attention :
 *  设计动机与公式推导见 gimbal.hpp 的文件头，本文件只写实现。
 *  控制律（与 V1 逐拍等价，已用 host 侧等价性测试验证）：
 *    yaw   : 连续角 c_k = c_{k-1} + wrap(m_k - m_{k-1})；
 *            目标展开 t_k = c_k + wrap(T - m_k)；输出 u = PID(t_k - c_k, c_k)
 *    pitch : 目标先钳位，输出 u = PID(clamp(T) - m, m)
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "gimbal.hpp"

#include <cmath>

#include "imu.hpp"
#include "motor.hpp"
#include "pid.hpp"

namespace robot {

namespace {

/* 把两拍角度差归一到 [-π, π]：
 * 跨零时真实转动是小量，直接相减会得到 ±2π 的假跳变。 */
float WrapDelta(float delta)
{
  return NormalizeAngleRad(delta);
}

}  // namespace

/* ======================= 连续角跟踪器 ======================= */

float Gimbal::AngleTracker::push(float angle)
{
  if (!primed_) {
    /* 第一拍没有参考，连续角就等于原始角 */
    value_ = angle;
    primed_ = true;
  } else {
    /* 只累加相邻两拍的**差值**，所以跨 ±π 不会跳 2π */
    value_ += WrapDelta(angle - last_);
  }
  last_ = angle;
  return value_;
}

/* ======================= 生命周期 ======================= */

void Gimbal::init(hello_world::motor::Motor *yaw_motor,
                  hello_world::motor::Motor *pitch_motor,
                  hello_world::pid::Pid *yaw_pid,
                  hello_world::pid::Pid *pitch_pid, const Config &cfg)
{
  yaw_motor_ = yaw_motor;
  pitch_motor_ = pitch_motor;
  yaw_pid_ = yaw_pid;
  pitch_pid_ = pitch_pid;
  cfg_ = cfg;

  target_ = GimbalAngle{};
  actual_ = GimbalAngle{};
  enabled_ = false;
  started_ = false;

  yaw_tracker_.invalidate();
  yaw_cont_ = 0.0f;

  /* imu_ 由 setImu() 注入，这里不清指针（否则"先 setImu 再 init"会丢 IMU） */
  last_absolute_ = false;
  last_feedback_yaw_ = 0.0f;
  feedback_switched_ = false;

  yaw_out_ = 0.0f;
  pitch_out_ = 0.0f;

  /* 清掉 PID 里可能残留的历史积分，避免一上电就出力 */
  if (yaw_pid_ != nullptr) {
    yaw_pid_->reset();
  }
  if (pitch_pid_ != nullptr) {
    pitch_pid_->reset();
  }
}

void Gimbal::setTarget(const GimbalAngle &target)
{
  target_ = target;
  enabled_ = true;  /* 急停后重新给目标即恢复控制 */
  started_ = true;
}

void Gimbal::stop(void)
{
  enabled_ = false;
  /* 目标跟随当前实际角：重新 enable 时误差约为 0，不会突跳 */
  target_ = actual_;

  /* 必须在这里直接清电机输入：上层急停分支调完 stop() 就 return，
   * 永远走不到 update() 里的清零点（考核说明 1.3 一票否决项）。 */
  if (yaw_motor_ != nullptr) {
    yaw_motor_->setInput(0.0f);
  }
  if (pitch_motor_ != nullptr) {
    pitch_motor_->setInput(0.0f);
  }
  if (yaw_pid_ != nullptr) {
    yaw_pid_->reset();
  }
  if (pitch_pid_ != nullptr) {
    pitch_pid_->reset();
  }
  yaw_out_ = 0.0f;
  pitch_out_ = 0.0f;
}

/* ======================= 周期入口 ======================= */

void Gimbal::update(float dt)
{
  if (!(dt > 0.0f)) {
    return;
  }

  if (!enabled_) {
    refreshWhenIdle();
    return;
  }

  updateYawAxis(dt);
  updatePitchAxis(dt);
}

void Gimbal::refreshWhenIdle(void)
{
  /* 未使能也要刷新 actual_，否则 stop() 里"目标跟随实际"会停在旧值。
   * 但从未启用过（刚开机）时不发帧：那时电机还没使能，发 0 力矩只白占总线。 */
  if (yaw_motor_ != nullptr) {
    if (started_) {
      yaw_motor_->setInput(0.0f);
    }
    actual_.yaw = yaw_motor_->angle();
  }
  if (pitch_motor_ != nullptr) {
    if (started_) {
      pitch_motor_->setInput(0.0f);
    }
    actual_.pitch = pitch_motor_->angle();
  }
}

void Gimbal::updateYawAxis(float dt)
{
  if (yaw_motor_ == nullptr) {
    return;
  }

  const float joint_yaw = yaw_motor_->angle();  /* 关节角：上报用，也是退路 */

  /* 反馈源：绝对角只在"配置开启 + IMU 已就绪"时生效，否则退回关节角 */
  const bool absolute =
      cfg_.absolute_yaw && (imu_ != nullptr) && imu_->isReady();
  const float measure = absolute ? imu_->yaw() : joint_yaw;

  /* 反馈源切换的那一拍：两种量纲的差不是真实转动，必须重起连续角；
   * 同时把目标对齐到新反馈（绝对角与关节角可能差很多，不对齐就是一次猛甩），
   * 并置一次性标志让上层也去对齐它自己的目标。 */
  if (absolute != last_absolute_) {
    yaw_tracker_.force(measure);
    yaw_cont_ = measure;
    target_.yaw = measure;
    last_absolute_ = absolute;
    feedback_switched_ = true;
  }

  yaw_cont_ = yaw_tracker_.push(measure);
  last_feedback_yaw_ = measure;

  /* 目标展开到"与当前连续角最近的等价角"，误差因此永远是最短路径 */
  const float target_cont = yaw_cont_ + WrapDelta(target_.yaw - measure);
  yaw_out_ = driveAxis(yaw_pid_, target_cont, yaw_cont_, dt);
  yaw_motor_->setInput(yaw_out_);

  /* 上报的永远是**归一化关节角**（G2C 里 yaw 的语义是"云台相对底盘"）。
   * 连续转多圈后归一化再打包，int16 才不会在 ±3.2767 rad 处饱和。 */
  actual_.yaw = NormalizeAngleRad(joint_yaw);
}

void Gimbal::updatePitchAxis(float dt)
{
  if (pitch_motor_ == nullptr) {
    return;
  }

  const float measure = pitch_motor_->angle();
  /* 目标先钳到软件限位；实际若已越界，误差反向，PID 自然往回拉 */
  const float target = limitPitch(target_.pitch);

  pitch_out_ = driveAxis(pitch_pid_, target, measure, dt);
  pitch_motor_->setInput(pitch_out_);

  actual_.pitch = measure;
}

/* ======================= 内部工具 ======================= */

float Gimbal::limitPitch(float angle) const
{
  if (angle < cfg_.pitch_min) {
    return cfg_.pitch_min;
  }
  if (angle > cfg_.pitch_max) {
    return cfg_.pitch_max;
  }
  return angle;
}

float Gimbal::driveAxis(hello_world::pid::Pid *pid, float target, float measure,
                        float dt) const
{
  if (pid == nullptr) {
    return 0.0f;
  }
  /* PID 接口约定：calc(误差, 测量值, dt)，微分作用于测量值 */
  return pid->calc(target - measure, measure, dt);
}

bool Gimbal::atTarget(float tol) const
{
  const float yaw_err = fabsf(NormalizeAngleRad(target_.yaw - actual_.yaw));
  const float pitch_err = fabsf(target_.pitch - actual_.pitch);
  return (yaw_err <= tol) && (pitch_err <= tol);
}

}  // namespace robot
