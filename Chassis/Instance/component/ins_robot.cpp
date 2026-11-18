/**
 *******************************************************************************
 * @file      :ins_robot.cpp
 * @brief     : 底盘板顶层状态机与底盘运动模块实例化
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 完成实例化
 *******************************************************************************
 * @attention :
 *  这里只负责静态定义并返回地址；各模块之间的注册关系由 Task/component/
 *  main_task.cpp 的 MainTaskInit() 按依赖顺序完成（电机 → PID → 逆解器 → 底盘 → Robot）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_robot.hpp"

#include "chassis.hpp"
#include "robot.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

namespace {

/* 静态存放底盘运动模块与顶层状态机 */
robot::Chassis g_chassis;
robot::Robot g_robot;

}  // namespace

/* Exported function definitions ---------------------------------------------*/
robot::Robot *GetRobot()
{
  return &g_robot;
}

robot::Chassis *GetChassis()
{
  return &g_chassis;
}

/* Private function definitions ----------------------------------------------*/
