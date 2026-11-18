/**
 *******************************************************************************
 * @file      :ins_comm.hpp
 * @brief     : 遥控器（DT7）解码对象 + 在线状态管理
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 增加遥控器在线判定接口
 *******************************************************************************
 * @attention :
 *  分层关系：Task 层的 UART 回调负责「收数据 + 调 decode + 通知已收到」，
 *  RobotModules 层通过 IsRemoteControlOnline() 判断遥控器是否有效，
 *  两边都只依赖 Instance 层，不互相依赖。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_INSTANCE_INS_COMM_HPP_
#define CHASSIS_INSTANCE_INS_COMM_HPP_

/* Includes ------------------------------------------------------------------*/
#include "DT7.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取遥控器解码对象（DT7）单例。 */
hello_world::remote_control::DT7 *GetRemoteControl();

/**
 * @brief       通知「刚收到一帧有效 DBUS 数据」
 * @retval      None
 * @note        由 Task 层的 HAL_UARTEx_RxEventCallback 在 decode() 之后调用
 */
void NotifyRemoteControlReceived(void);

/**
 * @brief       遥控器是否在线
 * @retval      true 表示在阈值时间内收到过有效 DBUS 帧
 * @note        离线时状态机必须保持零输出（考核说明 1.3「失效即归零」）
 */
bool IsRemoteControlOnline(void);

#endif /* CHASSIS_INSTANCE_INS_COMM_HPP_ */
