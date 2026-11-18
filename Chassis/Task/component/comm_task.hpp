/**
 *******************************************************************************
 * @file      :comm_task.hpp
 * @brief     : 通讯任务（CAN 收发 + DBUS 接收 + IWDG 喂狗）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 增加控制周期存活通知
 *******************************************************************************
 * @attention :
 *  None
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_TASK_COMM_TASK_HPP_
#define CHASSIS_TASK_COMM_TASK_HPP_

/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
/* Exported function prototypes ----------------------------------------------*/
void CommTaskInit(void);
void CommTask(void);

/**
 * @brief       通知「本周期控制流程已经完整跑完」
 * @retval      None
 * @note        由 MainTask() 在每个 1ms 周期末尾调用。CommTask() 只有在收到该通知后
 *              才刷新 IWDG，从而保证「控制流程卡死 → 看门狗超时复位」一定成立。
 */
void NotifyMainTaskAlive(void);
#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_TASK_COMM_TASK_HPP_ */
