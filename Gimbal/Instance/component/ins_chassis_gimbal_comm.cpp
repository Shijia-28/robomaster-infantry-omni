/**
 *******************************************************************************
 * @file      :ins_chassis_gimbal_comm.cpp
 * @brief     : 云台板双板通讯对象实例化（静默版）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 完成实例化 + CAN ID 选定
 *******************************************************************************
 * @attention :
 *  CAN ID 与底盘板严格对称（两板必须一致）：
 *    C2G（底盘 → 云台）= 0x210  ← 云台板接收
 *    G2C（云台 → 底盘）= 0x211  ← 云台板发送
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

/* 本板是云台板：发送 G2C（0x211），接收 C2G（0x210） */
constexpr uint32_t kTxId = robot::GimbalChassisComm::kGimbalTxId;
constexpr uint32_t kRxId = robot::GimbalChassisComm::kChassisTxId;

/* 函数内静态存放本板双板通讯对象（避免 main() 之前构造） */
robot::GimbalChassisComm *CommInstance(void)
{
  static robot::GimbalChassisComm comm{robot::GimbalChassisComm::CodePart::Gimbal,
                                       kTxId, kRxId};
  return &comm;
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/
robot::GimbalChassisComm *GetGimbalChassisComm()
{
  return CommInstance();
}

/* Private function definitions ----------------------------------------------*/
