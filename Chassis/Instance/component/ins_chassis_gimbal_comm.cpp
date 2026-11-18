/**
 *******************************************************************************
 * @file      :ins_chassis_gimbal_comm.cpp
 * @brief     : 底盘板双板通讯对象实例化
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 完成实例化 + CAN ID 选定
 *******************************************************************************
 * @attention :
 *  CAN ID 约定（两板必须一致，避开全部电机帧）：
 *    C2G（底盘 → 云台）= 0x210
 *    G2C（云台 → 底盘）= 0x211
 *  电机占用：M3508 控制 0x200 / 0x1FF，反馈 0x201~0x208；
 *            DM-J4310 发送 0x01~0x0F，反馈 0x11~0x1F。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_chassis_gimbal_comm.hpp"

#include "gimbal_chassis_comm.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

namespace {

/* 本板是底盘板：发送 C2G（0x210），接收 G2C（0x211） */
constexpr uint32_t kTxId = robot::GimbalChassisComm::kChassisTxId;
constexpr uint32_t kRxId = robot::GimbalChassisComm::kGimbalTxId;

/* 静态存放本板双板通讯对象 */
robot::GimbalChassisComm g_gimbal_chassis_comm{
    robot::GimbalChassisComm::CodePart::Chassis, kTxId, kRxId};

}  // namespace

/* Exported function definitions ---------------------------------------------*/
robot::GimbalChassisComm *GetGimbalChassisComm()
{
  return &g_gimbal_chassis_comm;
}

/* Private function definitions ----------------------------------------------*/
