/**
 *******************************************************************************
 * @file      :comm_task.cpp
 * @brief     : 通讯任务实现（CAN 收发 + DBUS 接收 + IWDG 喂狗）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 完成编写
 *******************************************************************************
 * @attention :
 *  1. 本文件负责：
 *       CommTaskInit  CAN 滤波器 + HAL_CAN_Start + 接收中断使能
 *                     USART3 空闲 + DMA 接收（18 字节 DBUS）
 *                     IWDG 首次刷新
 *       CommTask      1ms 周期：电机控制帧合并发送 + 双板通讯帧发送 + 条件喂狗
 *       UART 回调     收满 18 字节 → DT7.decode() → 重启接收
 *  2. 喂狗策略（考核说明 4.10）：只有在 MainTask() 完整跑完一个周期后才刷新 IWDG。
 *     控制流程卡死 → 没有 MainTask 通知 → 不喂狗 → 看门狗在约 300ms 内复位。
 *  3. 「电机 / 双板通讯接在哪路 CAN」由下面两个常量决定，实机接线不同时只改这两行；
 *     接收侧在 src/HW_can.cpp 中按帧 ID 路由，两路总线都会尝试分发，不怕接反。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "comm_task.hpp"

#include "HW_can.hpp"
#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "usart.h"

#include "DT7.hpp"
#include "debug_snapshot.hpp"
#include "gimbal_chassis_comm.hpp"
#include "ins_all.hpp"
#include "motor_M3508.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

namespace {

/* 双板通讯的 CAN 总线。实车约定：**CAN1 = 双板线，CAN2 = 电机**。
 * ⚠ 接收侧（HW_can.cpp 的 DispatchCanFrame）两路都会按帧 ID 路由，所以「收得到、
 *   发不出去」是这里写反的典型现象：对方发的帧你收得到，你发的帧对面永远收不到
 *   → 对面一直判你离线。改接线时这里必须与另一块板同时改。 */
CAN_HandleTypeDef *const kCommCan = &hcan1;
/* 底盘 4 个轮电机的 CAN 总线（与 kCommCan 相反） */
CAN_HandleTypeDef *const kMotorCan = &hcan2;

/* 轮电机数量 */
constexpr int kWheelMotorNum = 4;

/* M3508 两种控制帧 ID：ID 1~4 → 0x200，ID 5~8 → 0x1FF */
constexpr uint32_t kMotorTxIds[2] = {0x200u, 0x1FFu};

/* DBUS 接收缓冲（18 字节定长） */
uint8_t g_dbus_rx_buf[hello_world::remote_control::kRcRxDataLen];

/* MainTask 完整执行次数，用于喂狗判断 */
volatile uint32_t g_main_task_alive_cnt = 0u;

/* CommTask 上一次看到的 MainTask 计数 */
uint32_t g_last_main_task_cnt = 0u;

/* ---------------- 内部实现 ---------------- */

/**
 * @brief       把同一控制帧的电机合并成一帧后发送
 * @param        tx_id: 控制帧 ID（0x200 或 0x1FF）
 * @retval       None
 * @note        每个电机只往自己那 2 个字节写数据，所以必须先把 8 字节清零再合并
 */
void SendMotorFrame(uint32_t tx_id)
{
  uint8_t tx_data[8] = {0u};
  bool has_motor = false;

  for (int i = 0; i < kWheelMotorNum; ++i) {
    hello_world::motor::Motor *motor = GetMotorWheel(i);
    if (motor == nullptr) {
      continue;
    }
    if (hello_world::motor::M3508::GetTxId(motor->id()) != tx_id) {
      continue;
    }
    motor->encode(tx_data);
    has_motor = true;
  }

  if (has_motor) {
    CAN_Send_Msg(kMotorCan, tx_data, tx_id, 8u);
  }
}

/**
 * @brief       发送双板通讯帧（底盘 → 云台）
 * @retval       None
 * @note        None
 */
void SendGimbalChassisFrame(void)
{
  robot::GimbalChassisComm *comm = GetGimbalChassisComm();
  if (comm == nullptr) {
    return;
  }

  uint8_t tx_data[robot::kCommFrameLen] = {0u};
  comm->encode(tx_data);
  CAN_Send_Msg(kCommCan, tx_data, comm->tx_id(), robot::kCommFrameLen);
}

/**
 * @brief       启动 / 重启遥控器串口接收
 * @retval       None
 * @note        关闭 DMA 半传输中断，只用「空闲 + 满传输」事件，减少中断负担
 */
void StartDbusReceive(void)
{
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dbus_rx_buf,
                               hello_world::remote_control::kRcRxDataLen);
  /* 关闭 DMA 半传输中断（DMA_IT_HT），只用「空闲 + 满传输」事件，减少中断负担 */
  if (huart3.hdmarx != nullptr) {
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
  }
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/
extern "C" {

void CommTaskInit(void)
{
  /* 1. CAN 滤波器：全收（CAN1 → FIFO0，CAN2 → FIFO1） */
  CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);

  /* 2. 启动 CAN 并打开接收中断（CanFilter_Init 只配滤波器，不会让 CAN 离开初始化态）。
   *    这里不把启动失败当作致命错误：失败时电机帧 / 通讯帧发不出去，状态机会因为
   *    「双板通讯离线」而一直保持零输出，仍然是安全的，也不会把板子卡死在 Error_Handler。 */
  (void)HAL_CAN_Start(&hcan1);
  (void)HAL_CAN_Start(&hcan2);

  /* 3. 预热 4 个轮电机：让对象构造发生在**使能接收中断之前**，
   *    否则中断里首次 GetMotorWheel() 会在中断上下文跑构造与断言。 */
  for (int i = 0; i < kWheelMotorNum; ++i) {
    (void)GetMotorWheel(i);
  }
  /* 双板通讯对象同理预热（构造函数会读 GetTickMs 初始化在线时间戳） */
  (void)GetGimbalChassisComm();

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  /* 4. 启动遥控器 USART3 空闲 + DMA 接收 */
  StartDbusReceive();

  /* 5. 初始化完成，先喂一次狗（后面由 CommTask 按条件刷新） */
  HAL_IWDG_Refresh(&hiwdg);
}

void CommTask(void)
{
  /* 1. 电机控制帧：同帧电机合并后一次发送，避免 4 台电机各发一帧 */
  SendMotorFrame(kMotorTxIds[0]);
  SendMotorFrame(kMotorTxIds[1]);

  /* 2. 双板通讯帧 */
  SendGimbalChassisFrame();

  /* 3. 条件喂狗：只有 1ms 控制流程完整跑过之后才刷新 IWDG。
   *    若 MainTask / Robot::update 卡死，计数不再变化 → 停止喂狗 → 看门狗复位。 */
  const uint32_t main_task_cnt = g_main_task_alive_cnt;
  if (main_task_cnt != g_last_main_task_cnt) {
    g_last_main_task_cnt = main_task_cnt;
    HAL_IWDG_Refresh(&hiwdg);
  }

  /* 4. 刷新调试快照（只写全局结构体，不参与控制；Ozone 里 watch debug::g_snapshot） */
  debug::Update(g_dbus_rx_buf);
}

void NotifyMainTaskAlive(void)
{
  /* 注意：volatile 变量不能用 ++，这里显式读-改-写 */
  g_main_task_alive_cnt = g_main_task_alive_cnt + 1u;
}

/**
 * @brief   UART 接收事件回调（空闲 / 满传输）
 * @param   huart: 串口句柄
 * @param   Size: 本次接收到的字节数
 * @retval  None
 * @note    收满 18 字节才认为是完整 DBUS 帧；无论是否解包成功都要重启接收，
 *          否则一次错帧就会让遥控器永久失联（进而一直零输出）。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart3) {
    if (Size == hello_world::remote_control::kRcRxDataLen) {
      hello_world::remote_control::DT7 *rc = GetRemoteControl();
      if (rc != nullptr) {
        rc->decode(g_dbus_rx_buf);
        NotifyRemoteControlReceived();
      }
    }
    StartDbusReceive();
  }
}

/**
 * @brief   UART 错误回调（溢出 / 噪声 / 帧错误）
 * @param   huart: 串口句柄
 * @retval  None
 * @note    中止并重启接收，保证遥控器链路可以从错误中恢复
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3) {
    HAL_UART_AbortReceive(huart);
    StartDbusReceive();
  }
}

}  // extern "C"

/* Private function definitions ----------------------------------------------*/
