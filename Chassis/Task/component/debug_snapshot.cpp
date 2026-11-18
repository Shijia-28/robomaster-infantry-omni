/**
 *******************************************************************************
 * @file      : debug_snapshot.cpp
 * @brief     : 调试快照实现（只写不读，不参与控制）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-18      Chassis        1. 建立快照结构（全局结构体集中暴露关键量）
 *******************************************************************************
 * @attention :
 *  本文件只做「把变量拷进全局结构体」这一件事，不改变任何控制量、不做任何判断。
 *  排查方向类问题时，看 debug::g_snapshot 的 wheel_target / wheel_actual 符号即可
 *  分清「指令错了」还是「轮子没跟上」。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "debug_snapshot.hpp"

#include "chassis.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_comm.hpp"
#include "ins_robot.hpp"
#include "robot.hpp"
#include "tick.hpp"

namespace debug {

/* 全局快照（普通全局，Ozone 里 watch debug::g_snapshot 即可） */
Snapshot g_snapshot;

void Update(const uint8_t *dbus_raw)
{
  g_snapshot.tick_ms = hello_world::tick::GetTickMs();

  robot::Robot *r = GetRobot();
  if (r != nullptr) {
    g_snapshot.life_state = static_cast<uint8_t>(r->lifeState());
    g_snapshot.mode = static_cast<uint8_t>(r->chassisMode());
    g_snapshot.rc_online = r->isRcOnline() ? 1u : 0u;
    g_snapshot.run_enabled = r->isManualControlEnabled() ? 1u : 0u;
    g_snapshot.estop = r->isEmergency() ? 1u : 0u;
    g_snapshot.feedback_ok = r->isWheelFeedbackOk() ? 1u : 0u;
    g_snapshot.delta = r->delta();
    g_snapshot.gimbal_online = r->isGimbalOnline() ? 1u : 0u;
  }

  hello_world::remote_control::DT7 *rc = GetRemoteControl();
  if (rc != nullptr) {
    g_snapshot.rc_lv = rc->rc_lv();
    g_snapshot.rc_lh = rc->rc_lh();
    g_snapshot.rc_rh = rc->rc_rh();
    g_snapshot.rc_wheel = rc->rc_wheel();
  }

  robot::Chassis *c = GetChassis();
  if (c != nullptr) {
    g_snapshot.cmd_w = c->chassisSpeed().w;
    for (int i = 0; i < kDebugWheelNum; ++i) {
      g_snapshot.wheel_target[i] = c->wheelTargetVel()[i];
      g_snapshot.wheel_actual[i] = c->wheelActualVel()[i];
      g_snapshot.wheel_output[i] = c->wheelOutput()[i];
    }
  }

  /* 原始 DBUS 帧：排查拨轮/摇杆解码问题时可以直接看字节 */
  if (dbus_raw != nullptr) {
    for (int i = 0; i < 18; ++i) {
      g_snapshot.dbus_raw[i] = dbus_raw[i];
    }
  }
}

}  // namespace debug
