/**
 *******************************************************************************
 * @file      :ins_pid.cpp
 * @brief     : 云台 yaw / pitch 关节位置环 PID 实例化与参数
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 完成关节位置环实例化
 *******************************************************************************
 * @attention :
 *  1. 位置环的物理量：
 *       输入（误差） 关节角度误差，单位：rad
 *       输出         输出端力矩，单位：N·m（DM-J4310 的 TMAX = 7.5 N·m）
 *  2. 起调值 Kp = 8.0 / Ki = 0.40 / Kd = 0.60 / 积分限幅 2.0 / 输出限幅 7.5，
 *     实车按「更硬」的目标逐步上调，最终值见本文件下方的参数表。
 *     判据：不能"嗡嗡响"（Kp 过大），也不能"软绵绵追不上"（Kp 过小）。
 *  3. 微分作用于**测量值**（Pid 默认），避免目标阶跃时产生微分冲击。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_pid.hpp"

#include "pid.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

namespace {

/* ===== 关节位置环参数（力矩量纲）=====
 * 第二轮调硬（2026-11-18 实车）：8.0/0.40/0.60 → 10.5/0.50/0.80 → 现在 12.0/0.55/0.92。
 *   - 刚性 ∝ Kp：抗扰刚度、跟随到位速度都由它决定；
 *   - 阻尼 ∝ Kd：必须与 Kp **同比**放大，保持 kd/kp ≈ 0.077，否则"只加 Kp"必然嗡嗡响；
 *   - Ki 小幅上调，吃掉重力 / 线缆拉力造成的固定偏差。
 *
 * 调试阶梯（从这个值往下退，一次只改一个）：
 *   嗡嗡响 / 高频抖        → Kp 12.0 → 11.0，Kd 保持 0.92
 *   大角度甩到位后来回摆   → Kd 0.92 → 1.05（Kp 不动）
 *   静止时缓慢下坠 / 爬行  → Ki 0.55 → 0.70
 *   手掰还嫌软             → Kp 13.0 / Kd 1.00（这是上限，不要再往上）
 *
 * ⚠ out_lim 必须保持 7.5 N·m（DM-J4310 的 TMAX），加大会烧电机。 */
constexpr float kJointPidKp = 12.0f;  ///< 比例系数，单位：N·m / rad
constexpr float kJointPidKi = 0.55f;  ///< 积分系数，单位：N·m / (rad·s)
constexpr float kJointPidKd = 0.92f;  ///< 微分系数，单位：N·m·s / rad
constexpr float kJointPidILim = 2.8f; ///< 积分限幅，单位：N·m
constexpr float kJointPidOutLim = 7.5f;  ///< 输出限幅，单位：N·m（DM-J4310 TMAX，不可改）

}  // namespace

/* Exported function definitions ---------------------------------------------*/
hello_world::pid::Pid *GetPidMotorYaw()
{
  static hello_world::pid::Pid pid(kJointPidKp, kJointPidKi, kJointPidKd,
                                   kJointPidILim, kJointPidOutLim);
  return &pid;
}

hello_world::pid::Pid *GetPidMotorPitch()
{
  static hello_world::pid::Pid pid(kJointPidKp, kJointPidKi, kJointPidKd,
                                   kJointPidILim, kJointPidOutLim);
  return &pid;
}

/* Private function definitions ----------------------------------------------*/
