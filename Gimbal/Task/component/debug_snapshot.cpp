/**
 *******************************************************************************
 * @file      :debug_snapshot.cpp
 * @brief     : 云台板调试快照实现（只写不读，不参与控制）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-18      Chassis        1. 建立快照结构
 *******************************************************************************
 * @attention :
 *  本文件只做「把变量拷进全局结构体」这一件事，不改变任何控制量、不做任何判断。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "debug_snapshot.hpp"

#include "gimbal.hpp"
#include "gimbal_chassis_comm.hpp"
#include "ins_all.hpp"
#include "robot.hpp"
#include "tick.hpp"

namespace debug {

Snapshot g_snapshot;

void Update(const uint8_t *dbus_raw)
{
  g_snapshot.tick_ms = hello_world::tick::GetTickMs();

  robot::Robot *r = GetRobot();
  if (r != nullptr) {
    g_snapshot.life_state = static_cast<uint8_t>(r->lifeState());
    g_snapshot.mode = static_cast<uint8_t>(r->mode());
    g_snapshot.rc_online = r->isRcOnline() ? 1u : 0u;
    g_snapshot.run_enabled = r->isManualControlEnabled() ? 1u : 0u;
    g_snapshot.estop = r->isEmergency() ? 1u : 0u;
    g_snapshot.imu_ready = r->isImuReady() ? 1u : 0u;
    g_snapshot.target_yaw = r->targetAngle().yaw;
    g_snapshot.target_pitch = r->targetAngle().pitch;
  }

  hello_world::remote_control::DT7 *rc = GetRemoteControl();
  if (rc != nullptr) {
    g_snapshot.rc_lv = rc->rc_lv();
    g_snapshot.rc_lh = rc->rc_lh();
    g_snapshot.rc_rv = rc->rc_rv();
    g_snapshot.rc_rh = rc->rc_rh();
    g_snapshot.rc_l_switch = static_cast<uint8_t>(rc->rc_l_switch());
    g_snapshot.rc_r_switch = static_cast<uint8_t>(rc->rc_r_switch());
  }

  robot::Gimbal *g = GetGimbal();
  if (g != nullptr) {
    g_snapshot.gimbal_enabled = g->enabled() ? 1u : 0u;
    g_snapshot.absolute_active = g->absoluteActive() ? 1u : 0u;
    g_snapshot.actual_yaw = g->actual().yaw;
    g_snapshot.actual_pitch = g->actual().pitch;
    g_snapshot.yaw_out = g->yawOutput();
    g_snapshot.pitch_out = g->pitchOutput();
  }

  hello_world::motor::Motor *my = GetMotorYaw();
  if (my != nullptr) {
    g_snapshot.yaw_motor_angle = my->angle();
    g_snapshot.yaw_motor_vel = my->vel();
  }
  hello_world::motor::Motor *mp = GetMotorPitch();
  if (mp != nullptr) {
    g_snapshot.pitch_motor_angle = mp->angle();
    g_snapshot.pitch_motor_vel = mp->vel();
  }

  robot::GimbalChassisComm *comm = GetGimbalChassisComm();
  if (comm != nullptr) {
    g_snapshot.comm_online = comm->isOffline() ? 0u : 1u;
    const robot::GimbalChassisComm::ChassisData &cd = comm->chassis_data();
    g_snapshot.peer_life = static_cast<uint8_t>(cd.life_state);
    g_snapshot.peer_mode = static_cast<uint8_t>(cd.mode);
  }

  if (dbus_raw != nullptr) {
    for (int i = 0; i < 18; ++i) {
      g_snapshot.dbus_raw[i] = dbus_raw[i];
    }
  }

  /* ===== 瞄准诊断 ===== */
  if (r != nullptr) {
    g_snapshot.aim_yaw = r->aimYawNow();
    g_snapshot.aim_pitch = r->aimPitchNow();
    g_snapshot.input_armed = r->inputArmed() ? 1u : 0u;
    g_snapshot.working_entries = r->workingEntries();
  }
  if (comm != nullptr) {
    g_snapshot.comm_ever = comm->hasReceivedOnce() ? 1u : 0u;
  }
}

}  // namespace debug
