/**
 *******************************************************************************
 * @file      :ins_chassis_iksolver.cpp
 * @brief     : 底盘运动学逆解器实例化
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 完成实例化
 *******************************************************************************
 * @attention :
 *  几何参数使用考核说明 6.2 的实车数据（以实机测量为准）：
 *    轮半径 R      = 0.07786 m
 *    半轴距 L/2    = 0.15338 m
 *    半轮距 W/2    = 0.15338 m
 *    麦轮滚子角    = 45°
 *  若实车测量值不同，只需修改本文件构造参数（或在 ChassisIkSolver 里改默认值）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_chassis_iksolver.hpp"

#include "chassis_ik_solver.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

namespace {

/* 静态存放逆解器对象，几何参数取考核说明 6.2 的实车配置 */
hello_world::chassis_ik_solver::ChassisIkSolver g_chassis_ik_solver{
    hello_world::chassis_ik_solver::ChassisIkSolver::kDefaultWheelRadius,
    hello_world::chassis_ik_solver::ChassisIkSolver::kDefaultHalfWheelbase,
    hello_world::chassis_ik_solver::ChassisIkSolver::kDefaultHalfTrack};

}  // namespace

/* Exported function definitions ---------------------------------------------*/
hello_world::chassis_ik_solver::ChassisIkSolver *GetChassisIkSolver()
{
  return &g_chassis_ik_solver;
}

/* Private function definitions ----------------------------------------------*/
