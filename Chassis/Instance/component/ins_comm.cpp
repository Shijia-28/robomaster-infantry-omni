/**
 *******************************************************************************
 * @file      :ins_comm.cpp
 * @brief     : 遥控器（DT7）解码对象 + 在线状态管理
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 增加遥控器在线判定实现
 *******************************************************************************
 * @attention :
 *  DT7 只负责解码 18 字节 DBUS 数据；串口接收（空闲 + DMA）在
 *  Task/component/comm_task.cpp 中完成，解码入口是 HAL_UARTEx_RxEventCallback。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_comm.hpp"

#include "offline_checker.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

namespace {

/* 遥控器掉线阈值：DBUS 帧周期约 14ms，100ms 收不到即可判离线 */
constexpr uint32_t kRcOfflineThresMs = 100u;

/* 静态存放遥控器解码对象 */
hello_world::remote_control::DT7 g_remote_control;

/* 遥控器掉线检测器（初始即离线：上电后没收到过帧就绝不允许输出） */
hello_world::OfflineChecker g_rc_offline_checker{kRcOfflineThresMs};

}  // namespace

/* Exported function definitions ---------------------------------------------*/
hello_world::remote_control::DT7 *GetRemoteControl()
{
  return &g_remote_control;
}

void NotifyRemoteControlReceived(void)
{
  g_rc_offline_checker.update();
}

bool IsRemoteControlOnline(void)
{
  return !g_rc_offline_checker.isOffline();
}

/* Private function definitions ----------------------------------------------*/
