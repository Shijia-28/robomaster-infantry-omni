/**
 *******************************************************************************
 * @file      :debug_snapshot.hpp
 * @brief     : 云台板调试快照（只写不读，不参与任何控制逻辑）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-18      Chassis        1. 建立快照结构
 *******************************************************************************
 * @attention :
 *  1. 目的：把 1ms 周期里的关键量拷进一个**普通全局结构体**，调试器直接读，
 *     不需要在中断里下断点（下断点会让看门狗 600ms 后复位）。
 *  2. 只写不读：本结构体的数据不参与任何控制判断，加它不会改变控制行为。
 *  3. 「云台不动」时先看这里：
 *       is_rc_online  == 0  → 云台板没收到 DBUS（接收机没接到 USART3），永远进不了 Working
 *       is_imu_ready  == 0  → IMU 零漂没收敛 / 芯片没认出来，同样进不了 Working
 *       life_state    == 0  → 还在 Dead；== 1 表示自检中；== 2 才允许出力
 *       run_enabled   == 0  → 右拨杆不在运行档（Up）
 *       estop         == 1  → 急停 / 离线，两轴已卸力
 *       yaw_angle / pitch_angle 一直是 0 且不随手动变化 → 达妙电机没有反馈
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_TASK_DEBUG_SNAPSHOT_HPP_
#define GIMBAL_TASK_DEBUG_SNAPSHOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

namespace debug {

/* 调试快照（普通全局变量，调试器直接读） */
struct Snapshot {
  uint32_t tick_ms = 0u;   ///< 采样时刻，单位 ms

  /* 状态机 / 模式 */
  uint8_t life_state = 0u;  ///< 0 Dead / 1 Resurrection / 2 Working
  uint8_t mode = 0u;        ///< 0 kJoint / 1 kCartesian
  uint8_t rc_online = 0u;   ///< 遥控器是否在线
  uint8_t run_enabled = 0u; ///< 右拨杆是否在运行档
  uint8_t estop = 0u;       ///< 急停 / 保护
  uint8_t imu_ready = 0u;   ///< IMU 零漂是否收敛
  uint8_t gimbal_enabled = 0u;  ///< 关节环是否在闭环
  uint8_t absolute_active = 0u; ///< yaw 是否真的在用绝对角

  /* 遥控器原始量 */
  float rc_lv = 0.0f;   ///< 左摇杆竖直 [-1,1]
  float rc_lh = 0.0f;   ///< 左摇杆水平
  float rc_rv = 0.0f;   ///< 右摇杆竖直（pitch）
  float rc_rh = 0.0f;   ///< 右摇杆水平（yaw）
  uint8_t rc_l_switch = 0u;  ///< 左拨杆档位 0/1/2
  uint8_t rc_r_switch = 0u;  ///< 右拨杆档位 0/1/2

  /* 关节量 */
  float target_yaw = 0.0f;   ///< 目标关节角，rad
  float target_pitch = 0.0f;
  float actual_yaw = 0.0f;   ///< 实际上报的关节角（归一化），rad
  float actual_pitch = 0.0f;
  float yaw_out = 0.0f;      ///< yaw 力矩指令，N·m
  float pitch_out = 0.0f;    ///< pitch 力矩指令，N·m

  /* 电机反馈角度（达妙没通信时会一直是 0） */
  float yaw_motor_angle = 0.0f;
  float pitch_motor_angle = 0.0f;
  float yaw_motor_vel = 0.0f;
  float pitch_motor_vel = 0.0f;

  /* 双板通讯 */
  uint8_t comm_online = 0u;   ///< 收到过对板 C2G 帧且未超时
  uint8_t peer_life = 0u;     ///< 对板 life_state
  uint8_t peer_mode = 0u;     ///< 对板底盘模式

  /* 最近一帧 DBUS 原始 18 字节 */
  uint8_t dbus_raw[18] = {0};

  /* ===== 瞄准诊断（排查「推右摇杆云台不动」）===== */
  float aim_yaw = 0.0f;       ///< 本周期**实际使用**的瞄准输入 [-1,1]（本地或对板）
  float aim_pitch = 0.0f;     ///< 同上
  uint8_t input_armed = 0u;   ///< 是否已解锁目标角积分（0=卡在回中等待）
  uint8_t comm_ever = 0u;     ///< 是否曾经收到过对板帧
  uint16_t reserved = 0u;
  uint32_t working_entries = 0u;  ///< 进入 Working 的次数（>1 说明状态在抖、反复重置武装）
};

/* 全局快照实例 */
extern Snapshot g_snapshot;

/**
 * @brief       刷新调试快照
 * @param       dbus_raw: 最近一帧 DBUS 原始数据（18 字节），可为 nullptr
 * @retval      None
 * @note        由 CommTask 每个 1ms 周期调用一次
 */
void Update(const uint8_t *dbus_raw);

}  // namespace debug

#endif /* GIMBAL_TASK_DEBUG_SNAPSHOT_HPP_ */
