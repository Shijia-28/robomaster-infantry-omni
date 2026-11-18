/**
 *******************************************************************************
 * @file      :main_task.cpp
 * @brief     : 任务调度（一次性初始化 + 1ms 周期入口 + 定时器中断分发）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 完成编写
 *******************************************************************************
 * @attention :
 *  1. MainTaskInit() 必须由 main() 在 while(1) 之前、HAL_TIM_Base_Start_IT(&htim6)
 *     之前调用一次 —— 否则 1ms 中断可能在模块还没初始化完就进来。
 *  2. HAL_TIM_PeriodElapsedCallback 必须先判断中断来自 htim6：本工程 TIM2 / TIM4
 *     也开了更新中断（CubeMX 生成），不判断会把它们误当控制周期。
 *     ⚠ TIM4 在本板上只做 PWM（PD14），CubeMX 给的 htim4.Init.Period = 0，
 *       又对它调了 HAL_TIM_Base_Start_IT，相当于每 1us 溢出一次；
 *       这里在第一次进入时把它的更新中断关掉。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"

#include "comm_task.hpp"

#include "gimbal.hpp"
#include "imu.hpp"
#include "ins_all.hpp"
#include "iwdg.h"
#include "robot.hpp"
#include "tim.h"

/* Private macro -------------------------------------------------------------*/
/* Private constants --------------------------------------------------------- */

namespace {

/* 云台板控制周期，秒（TIM6 = 1 kHz） */
constexpr float kControlPeriodSec = 0.001f;

/* 是否已完成一次性初始化 */
bool g_main_task_inited = false;

}  // namespace

/* Exported function definitions ---------------------------------------------*/

extern "C" {

void MainTaskInit(void)
{
  if (g_main_task_inited) {
    return;
  }
  g_main_task_inited = true;

  /* 0. 先喂一次狗：后面的 IMU 芯片初始化是阻塞流程 */
  HAL_IWDG_Refresh(&hiwdg);

  /* 1. IMU：芯片自检 + 零漂标定启动（阻塞，必须在开中断之前完成）。
   *    零漂真正收敛靠 1ms 周期里的 Imu::update()。 */
  hello_world::imu::Imu *imu = GetImu();
  if (imu != nullptr) {
    imu->init();
    HAL_IWDG_Refresh(&hiwdg);
  }

  /* 2. 云台运动模块：注入两个关节电机与两路位置环 PID。
   *    ⚠ 关节限位与 Robot 的目标角钳位必须是同一组数（两处都钳，谁都不能漏）。 */
  robot::Robot::Config robot_cfg;
  robot::Gimbal::Config gimbal_cfg;
  gimbal_cfg.absolute_yaw = robot_cfg.absolute_yaw;
  gimbal_cfg.pitch_min = robot_cfg.pitch_min;
  gimbal_cfg.pitch_max = robot_cfg.pitch_max;

  robot::Gimbal *gimbal = GetGimbal();
  if (gimbal != nullptr) {
    gimbal->init(GetMotorYaw(), GetMotorPitch(), GetPidMotorYaw(),
                 GetPidMotorPitch(), gimbal_cfg);
    gimbal->setImu(imu);
    gimbal->stop();  /* 上电先卸力，保证第一拍就是零输出 */
  }

  /* 3. 顶层状态机：套用配置并注册各模块 */
  robot::Robot *robot = GetRobot();
  if (robot != nullptr) {
    robot->init(robot_cfg);
    robot->registerGimbal(gimbal);
    robot->registerImu(imu);
    robot->registerRemoteControl(GetRemoteControl());
    robot->registerGimbalChassisComm(GetGimbalChassisComm());
  }

  /* 4. 通讯层初始化：CAN 滤波器 + CAN 启动 + DBUS 接收 + 看门狗首次刷新 */
  CommTaskInit();
}

void MainTask(void)
{
  robot::Robot *robot = GetRobot();
  if (robot != nullptr) {
    robot->update(kControlPeriodSec);
  }

  /* 通知通讯任务「本周期控制流程已完整跑完」，CommTask 据此条件喂狗 */
  NotifyMainTaskAlive();
}

/**
 * @brief   定时器更新中断回调
 * @param   htim: 定时器句柄
 * @retval  None
 * @note    只有 htim6（1ms）驱动控制周期；htim4 / htim2 的更新中断在首次进入时关掉。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim6) {
    MainTask();
    CommTask();
    return;
  }

  if ((htim == &htim4) || (htim == &htim2)) {
    __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
  }
}

}  // extern "C"

/* Private function definitions ----------------------------------------------*/
