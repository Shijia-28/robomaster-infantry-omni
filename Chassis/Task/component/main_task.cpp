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
 *  1. MainTaskInit() 必须由 main() 在 while(1) 之前调用一次，并且必须在
 *     HAL_TIM_Base_Start_IT(&htim6) 之前调用 —— 否则 1ms 中断可能在模块还没初始化完
 *     就进来，访问到未注册的对象。
 *  2. MainTask() 是 1ms 周期入口，只调用 robot->update()，保持薄薄一层。
 *  3. HAL_TIM_PeriodElapsedCallback 里必须先判断中断来自 htim6：本工程 TIM2 / TIM4
 *     也开了更新中断（TIM2 用作 1MHz 计时、TIM4 用作 PWM），不判断会把它们误当控制周期。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"

#include "comm_task.hpp"

#include "chassis.hpp"
#include "imu.hpp"
#include "ins_all.hpp"
#include "iwdg.h"
#include "robot.hpp"
#include "tim.h"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

namespace {

/* 轮电机数量 */
constexpr int kWheelMotorNum = 4;

/* 是否已完成一次性初始化（防止重复调用导致重复注册 / 重复喂狗） */
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

  /* 0. 先喂一次狗：后面的 IMU 芯片初始化是阻塞流程，避免初始化期间被看门狗打断 */
  HAL_IWDG_Refresh(&hiwdg);

  /* 1. IMU：芯片自检 + 零漂标定启动（阻塞，必须在开中断之前完成）
   *    注意：零漂真正收敛是靠 1ms 周期里的 Imu::update()，这里只做芯片初始化。 */
  hello_world::imu::Imu *imu = GetImu();
  if (imu != nullptr) {
    imu->init();
    /* 芯片初始化是阻塞流程（软复位 + 寄存器回读，片选错时还会再试一次），
     * 中间再喂一次狗，确保不会在初始化期间被 300ms 的 IWDG 打断。 */
    HAL_IWDG_Refresh(&hiwdg);
  }

  /* 2. 底盘运动模块：按「电机 → PID → 逆解器」的顺序注册 */
  robot::Chassis *chassis = GetChassis();
  if (chassis != nullptr) {
    for (int i = 0; i < kWheelMotorNum; ++i) {
      chassis->registerMotor(i, GetMotorWheel(i));
      chassis->registerPid(i, GetPidMotorWheel(i));
    }
    chassis->registerIkSolver(GetChassisIkSolver());
    /* 上电先把所有电机指令清 0，保证第一拍就是零输出 */
    chassis->reset();
  }

  /* 3. 顶层状态机：注册各模块 */
  robot::Robot *robot = GetRobot();
  if (robot != nullptr) {
    robot->registerChassis(chassis);
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
    robot->update();
  }

  /* 通知通讯任务「本周期控制流程已完整跑完」，CommTask 据此条件喂狗 */
  NotifyMainTaskAlive();
}

/**
 * @brief   定时器更新中断回调
 * @param   htim: 定时器句柄
 * @retval  None
 * @note    1. 只有 htim6（1ms）才驱动控制周期；
 *          2. TIM4 在本工程里只用作 PWM 输出（PD14 / TIM4_CH3），CubeMX 生成的
 *             htim4.Init.Period = 0，而 main.c 又对它调用了 HAL_TIM_Base_Start_IT，
 *             相当于「每 1us 溢出一次」，会以 1MHz 的频率进中断把 CPU 打满。
 *             这里在第一次进来时把它的更新中断关掉（PWM 输出由 CCR/CCER 决定，
 *             不受更新中断影响），TIM2 同理（本工程用 HAL_GetTick 做时间基准，
 *             TIM2 的更新中断没有用处）。
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
