/**
 *******************************************************************************
 * @file      :ins_robot.cpp
 * @brief     : 云台板云台运动模块与顶层状态机实例化
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 完成实例化
 *******************************************************************************
 * @attention :
 *  这里只负责静态定义并返回地址；注册关系由 main_task.cpp 的 MainTaskInit()
 *  按「电机 → PID → Gimbal → Robot」的顺序完成。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_robot.hpp"

#include "gimbal.hpp"
#include "robot.hpp"

namespace {

/* 函数内静态：首次 Get 时构造（避免 main() 之前跑构造） */
robot::Gimbal *GimbalInstance(void)
{
  static robot::Gimbal gimbal;
  return &gimbal;
}

robot::Robot *RobotInstance(void)
{
  static robot::Robot robot;
  return &robot;
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/
robot::Robot *GetRobot()
{
  return RobotInstance();
}

robot::Gimbal *GetGimbal()
{
  return GimbalInstance();
}
