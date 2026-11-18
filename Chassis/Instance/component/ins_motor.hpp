/**
 *******************************************************************************
 * @file      :ins_motor.hpp
 * @brief     : 底盘轮电机实例 + 编码器反馈在线检测
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 增加轮电机反馈在线检测接口
 *******************************************************************************
 * @attention :
 *  「反馈在线」的用途：轮速环一旦拿不到编码器反馈，误差恒等于目标值，P/I 会把电流
 *  推到限幅，架空的车轮就会一路加速停不下来（实测现象：「轻轻给一点就飞速转，而且
 *  越转越快」）。因此 Robot 在允许出力前会检查四轮反馈是否都在线。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_INSTANCE_INS_MOTOR_HPP_
#define CHASSIS_INSTANCE_INS_MOTOR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "motor.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取第 idx 号轮电机单例（M3508，共 4 个，idx ∈ [0,4)）。 */
hello_world::motor::Motor *GetMotorWheel(int idx);

/**
 * @brief       通知「第 idx 号轮电机刚收到一帧反馈」
 * @param        idx: 轮序号（chassis_ik_solver::WheelIndex）
 * @retval       None
 * @note        由 CAN 接收分发的电机 decode 处调用（src/HW_can.cpp）
 */
void NotifyWheelFeedback(int idx);

/**
 * @brief       第 idx 号轮电机的反馈是否在线（阈值 100 ms）
 * @param        idx: 轮序号
 * @retval       true = 阈值时间内收到过反馈
 * @note        上电后从未收到过也返回 false
 */
bool IsWheelFeedbackOnline(int idx);

#endif /* CHASSIS_INSTANCE_INS_MOTOR_HPP_ */
