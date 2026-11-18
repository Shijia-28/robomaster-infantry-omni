/**
 *******************************************************************************
 * @file      : debug_snapshot.hpp
 * @brief     : 调试快照（只写不读，不参与任何控制逻辑）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-18      Chassis        1. 建立快照结构（全局结构体集中暴露关键量）
 *******************************************************************************
 * @attention :
 *  1. 目的：把 1 ms 周期里的关键量拷进一个**普通的全局结构体**，任何调试器都能直接
 *     看，不需要在中断里下断点（下断点会让看门狗 300 ms 后复位）。
 *  2. 只写不读：本文件的数据不参与任何控制判断，加它不会改变控制行为。
 *  3. 用法（Ozone / J-Link）：
 *       打开 Chassis/build/Debug/omni_chassis.elf → 连接 → 在 Watch 里加
 *       debug::g_snapshot
 *     然后展开看各个字段；Live Watch 模式下不会停 CPU。
 *
 *     最常看的几个：
 *       mode          0=分离 1=跟随 2=小陀螺
 *       rc_wheel      拨轮原始值（左推负、右推正）
 *       cmd_w         本周期实际使用的角速度指令（分离档 = rc_wheel 映射值）
 *       wheel_target  四个轮的目标轮速（FL/FR/RL/RR）
 *       wheel_actual  四个轮的实际轮速
 *     只要 target 与 actual 同号，就说明"轮子确实按要求在转"。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_TASK_DEBUG_SNAPSHOT_HPP_
#define CHASSIS_TASK_DEBUG_SNAPSHOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

namespace debug {

/* 轮子数量 */
constexpr int kDebugWheelNum = 4;

/* 调试快照（普通全局变量，Ozone 直接 watch） */
struct Snapshot {
  uint32_t tick_ms = 0u;      ///< 采样时刻，单位 ms

  /* 状态机 / 模式 */
  uint8_t life_state = 0u;    ///< 0 Dead / 1 Resurrection / 2 Working
  uint8_t mode = 0u;          ///< 0 分离 / 1 跟随 / 2 小陀螺
  uint8_t rc_online = 0u;     ///< 遥控器是否在线
  uint8_t run_enabled = 0u;   ///< 右拨杆是否在运行档
  uint8_t estop = 0u;         ///< 是否处于急停 / 保护
  uint8_t feedback_ok = 0u;   ///< 四轮反馈是否都在线

  /* 遥控器原始量 */
  float rc_lv = 0.0f;         ///< 左摇杆竖直 [-1,1]
  float rc_lh = 0.0f;         ///< 左摇杆水平 [-1,1]
  float rc_rh = 0.0f;         ///< 右摇杆水平 [-1,1]
  float rc_wheel = 0.0f;      ///< 拨轮 [-1,1]

  /* 云台 / 坐标系 */
  float delta = 0.0f;         ///< 云台相对底盘偏航角，单位 rad
  uint8_t gimbal_online = 0u; ///< 双板通讯是否在线

  /* 底盘指令与执行 */
  float cmd_w = 0.0f;         ///< 本周期实际使用的角速度指令，单位 rad/s
  float wheel_target[kDebugWheelNum] = {0.0f, 0.0f, 0.0f, 0.0f};  ///< 目标轮速 rad/s
  float wheel_actual[kDebugWheelNum] = {0.0f, 0.0f, 0.0f, 0.0f};  ///< 实际轮速 rad/s
  float wheel_output[kDebugWheelNum] = {0.0f, 0.0f, 0.0f, 0.0f};  ///< 电流指令 A

  /* 最近一帧 DBUS 原始 18 字节（排查遥控器解码问题用：直接看字节，不靠猜） */
  uint8_t dbus_raw[18] = {0};
};

/* 全局快照实例（唯一实例，Ozone 里直接 watch 这个名字） */
extern Snapshot g_snapshot;

/**
 * @brief       刷新调试快照
 * @param        dbus_raw: 最近一帧 DBUS 原始数据（18 字节），可为 nullptr
 * @retval      None
 * @note        由 CommTask 每个 1ms 周期调用一次；只读各模块、只写本结构体
 */
void Update(const uint8_t *dbus_raw);

}  // namespace debug

#endif /* CHASSIS_TASK_DEBUG_SNAPSHOT_HPP_ */
