/**
 *******************************************************************************
 * @file      : trace_main.cpp
 * @brief     : 云台模块等价性测试驱动器（host 侧）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-18      Chassis        1. 用同一组输入脚本驱动新旧实现，输出轨迹供比对
 *******************************************************************************
 * @attention :
 *  1. 本程序同时被旧实现（omni_skeleton）和新实现（omni_skeleton_rw）编译，
 *     两边输出逐行比对，用于证明重构"行为完全等价"。
 *  2. 输入脚本刻意覆盖所有分支：
 *       阶段 A  关节角连续过零（跨 ±π 多次）+ 目标大角度展开
 *       阶段 B  IMU 就绪 -> 反馈源从关节角切到绝对角（含无扰对齐）
 *       阶段 C  绝对角模式下底盘转动（IMU yaw 变、关节角反向变）
 *       阶段 D  pitch 目标越限（上下限都要撞到）
 *       阶段 E  stop() / setTarget() 反复启停（PID 复位与恢复）
 *       阶段 F  IMU 失就绪 -> 退回关节角
 *  3. 输出格式：每个周期一行 CSV，浮点用 %.9g 保证可逐字比对。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include <cmath>
#include <cstdio>

#include "gimbal.hpp"
#include "imu.hpp"
#include "motor.hpp"
#include "pid.hpp"

namespace {

constexpr float kPi = 3.14159265358979f;
constexpr float kDt = 0.001f;
constexpr int kCyclesPerStage = 800;

/* 测试用关节电机 */
hello_world::motor::Motor g_yaw_motor;
hello_world::motor::Motor g_pitch_motor;
hello_world::imu::Imu g_imu;

/* 把角度归一到 [-π, π)，与 module_state.hpp 的约定一致（这里独立实现，
 * 避免测试驱动本身依赖被测代码的归一化函数） */
float Wrap(float a)
{
  while (a >= kPi) {
    a -= 2.0f * kPi;
  }
  while (a < -kPi) {
    a += 2.0f * kPi;
  }
  return a;
}

void Emit(int cycle, robot::Gimbal &g)
{
  /* 一次性标志，必须在读其他量之前取（读到即清除，与上层用法一致） */
  const bool switched = g.consumeFeedbackSwitch();
  printf("%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%.9g,%.9g,%d\n", cycle,
         static_cast<double>(g.target().yaw), static_cast<double>(g.actual().yaw),
         static_cast<double>(g.target().pitch),
         static_cast<double>(g.actual().pitch),
         static_cast<double>(g.yawOutput()), static_cast<double>(g.pitchOutput()),
         static_cast<double>(g.yawContinuous()), g.absoluteActive() ? 1 : 0,
         static_cast<double>(g.yawFeedback()),
         static_cast<double>(g_yaw_motor.lastInput()), switched ? 1 : 0);
}

}  // namespace

int main(void)
{
  /* PID 用与实车一致的参数（关节位置环） */
  hello_world::pid::Pid yaw_pid(8.0f, 0.40f, 0.60f, 2.0f, 7.5f);
  hello_world::pid::Pid pitch_pid(8.0f, 0.40f, 0.60f, 2.0f, 7.5f);

  robot::Gimbal gimbal;
  robot::Gimbal::Config cfg;
  cfg.pitch_min = -0.40f;
  cfg.pitch_max = 0.60f;
  cfg.absolute_yaw = true;

  gimbal.init(&g_yaw_motor, &g_pitch_motor, &yaw_pid, &pitch_pid, cfg);
  gimbal.setImu(&g_imu);

  int cycle = 0;

  /* ---------- 阶段 A：关节角连续过零 4 圈多，目标给大角度 ---------- */
  for (int i = 0; i < kCyclesPerStage; ++i) {
    const float t = static_cast<float>(i) * kDt;
    g_yaw_motor.setAngle(Wrap(2.5f * t));            /* 2.5 rad/s 连续转 */
    g_pitch_motor.setAngle(0.10f * sinf(3.0f * t));  /* pitch 小幅摆动 */
    if (i == 0) {
      robot::GimbalAngle target{};
      target.yaw = 3.0f;   /* 与当前角形成大角度差，检验目标展开 */
      target.pitch = 0.05f;
      gimbal.setTarget(target);
    }
    gimbal.update(kDt);
    Emit(cycle++, gimbal);
  }

  /* ---------- 阶段 B：IMU 就绪，反馈源切到绝对角 ---------- */
  g_imu.setYaw(Wrap(g_yaw_motor.angle() + 1.20f));  /* 绝对角与关节角差 1.2rad */
  g_imu.setReady(true);
  for (int i = 0; i < kCyclesPerStage; ++i) {
    const float t = static_cast<float>(i) * kDt;
    g_yaw_motor.setAngle(Wrap(g_yaw_motor.angle() + 2.0f * kDt));
    g_pitch_motor.setAngle(0.10f * sinf(3.0f * t));
    g_imu.setYaw(Wrap(g_imu.yaw() + 2.0f * kDt));
    gimbal.update(kDt);
    Emit(cycle++, gimbal);
  }

  /* ---------- 阶段 C：绝对角模式下"底盘转动"（关节角反向变） ---------- */
  for (int i = 0; i < kCyclesPerStage; ++i) {
    g_yaw_motor.setAngle(Wrap(g_yaw_motor.angle() - 1.5f * kDt));  /* 底盘转 → 关节角反向 */
    g_imu.setYaw(Wrap(g_imu.yaw() + 1.5f * kDt));                  /* 云台保持世界朝向 */
    g_pitch_motor.setAngle(0.15f);
    gimbal.update(kDt);
    Emit(cycle++, gimbal);
  }

  /* ---------- 阶段 D：pitch 目标越限（上下限各撞一次） ---------- */
  {
    robot::GimbalAngle target = gimbal.target();
    target.pitch = 2.0f;  /* 远超上限 0.60 */
    gimbal.setTarget(target);
  }
  for (int i = 0; i < kCyclesPerStage / 2; ++i) {
    gimbal.update(kDt);
    Emit(cycle++, gimbal);
  }
  {
    robot::GimbalAngle target = gimbal.target();
    target.pitch = -2.0f;  /* 远超下限 -0.40 */
    gimbal.setTarget(target);
  }
  for (int i = 0; i < kCyclesPerStage / 2; ++i) {
    gimbal.update(kDt);
    Emit(cycle++, gimbal);
  }

  /* ---------- 阶段 E：stop() / setTarget() 反复启停 ---------- */
  for (int k = 0; k < 20; ++k) {
    for (int i = 0; i < 25; ++i) {
      gimbal.update(kDt);
      Emit(cycle++, gimbal);
    }
    gimbal.stop();
    for (int i = 0; i < 25; ++i) {
      gimbal.update(kDt);
      Emit(cycle++, gimbal);
    }
    robot::GimbalAngle target{};
    target.yaw = 0.30f * static_cast<float>(k);
    target.pitch = 0.02f * static_cast<float>(k);
    gimbal.setTarget(target);
  }

  /* ---------- 阶段 F：IMU 失就绪，退回关节角 ---------- */
  g_imu.setReady(false);
  for (int i = 0; i < kCyclesPerStage; ++i) {
    g_yaw_motor.setAngle(Wrap(g_yaw_motor.angle() + 1.0f * kDt));
    gimbal.update(kDt);
    Emit(cycle++, gimbal);
  }

  return 0;
}
