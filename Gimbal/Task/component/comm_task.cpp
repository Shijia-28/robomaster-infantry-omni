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
 *       CommTask      1ms 周期：两轴达妙控制帧 + 双板通讯帧发送 + 条件喂狗
 *       UART 回调     收满 18 字节 → DT7.decode() → 重启接收
 *  2. 【总线分配】实车约定：**CAN1 = 双板线，CAN2 = 电机**。
 *     云台板只在 CAN2 上发达妙控制帧（MIT 模式 tx_id = 电机 id，0x01 / 0x02），
 *     在 CAN1 上发 G2C（0x211）。接收侧在 src/HW_can.cpp 里两路都按 ID 路由。
 *  3. 【喂狗策略】只有在 MainTask() 完整跑完一个周期后才刷新 IWDG；
 *     控制流程卡死 → 没有通知 → 不喂狗 → 看门狗在约 600ms 内复位。
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
#include "motor_DaMiao.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

namespace {

/* ===== 总线分配（按实车接线，两轴不在同一条总线上）=====
  *       kCommCan       = &hcan1;   // G2C 双板帧
 *       kYawMotorCan   = &hcan1;   // ★ yaw 达妙电机也在 CAN1
 *       kPitchMotorCan = &hcan2;   // ★ pitch 达妙电机在 CAN2
 *   两轴不在同一条总线上！按「两轴都在 CAN2」写会导致 yaw 电机收不到任何指令。*/
CAN_HandleTypeDef *const kCommCan = &hcan1;
CAN_HandleTypeDef *const kYawMotorCan = &hcan1;
CAN_HandleTypeDef *const kPitchMotorCan = &hcan2;

/* 双板帧发送分频：1 ms 任务 × 10 = 100 Hz。
 * 对方离线阈值 100 ms，100 Hz 已有 10 帧冗余；CAN1 上还有 1 kHz 的 yaw 帧，
 * 没必要再拿 1 kHz 去挤。 */
constexpr uint32_t kCommTxDivider = 10u;

/* 关节电机数量 */
constexpr int kJointMotorNum = 2;

/* DBUS 接收缓冲（18 字节定长） */
uint8_t g_dbus_rx_buf[hello_world::remote_control::kRcRxDataLen];

/* MainTask 完整执行次数，用于喂狗判断 */
volatile uint32_t g_main_task_alive_cnt = 0u;

/* CommTask 上一次看到的 MainTask 计数 */
uint32_t g_last_main_task_cnt = 0u;

/* 双板帧发送分频计数 */
uint32_t g_comm_tx_div = 0u;

/**
 * @brief       发送一帧达妙控制帧
 * @param       motor: 电机对象
 * @retval      None
 * @note        encode() 内部会更新 tx_id()：MIT 模式 = 电机 id，
 *              使能 / 失能 / 清错命令 = 0x7FF
 */
void SendJointFrame(hello_world::motor::Motor *motor,
                    CAN_HandleTypeDef *can)
{
  if ((motor == nullptr) || (can == nullptr)) {
    return;
  }
  uint8_t tx_data[8] = {0u};
  if (!motor->encode(tx_data)) {
    return;
  }
  auto *dm = static_cast<hello_world::motor::DaMiao *>(motor);
  CAN_Send_Msg(can, tx_data, dm->tx_id(), 8u);
}

/**
 * @brief       发送双板通讯帧（云台 → 底盘）
 * @retval      None
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
 * @retval      None
 * @note        关闭 DMA 半传输中断，只用「空闲 + 满传输」事件，减少中断负担
 */
void StartDbusReceive(void)
{
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_dbus_rx_buf,
                               hello_world::remote_control::kRcRxDataLen);
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

  /* 2. 启动 CAN 并打开接收中断（CanFilter_Init 只配滤波器，不会让 CAN 离开初始化态） */
  (void)HAL_CAN_Start(&hcan1);
  (void)HAL_CAN_Start(&hcan2);

  /* 3. 预热电机与双板通讯对象：让构造发生在使能接收中断之前 */
  for (int i = 0; i < kJointMotorNum; ++i) {
    (void)((i == 0) ? GetMotorYaw() : GetMotorPitch());
  }
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
  /* 1. 两轴达妙控制帧（每轴一帧，1 kHz；MIT 模式下 tx_id 就是电机 id）
   *    ⚠ yaw 在 CAN1、pitch 在 CAN2，不要合成一条总线发。 */
  SendJointFrame(GetMotorYaw(), kYawMotorCan);
  SendJointFrame(GetMotorPitch(), kPitchMotorCan);

  /* 2. 双板通讯帧（G2C）：100 Hz 分频发送 */
  if (++g_comm_tx_div >= kCommTxDivider) {
    g_comm_tx_div = 0u;
    SendGimbalChassisFrame();
  }

  /* 3. 条件喂狗：只有 1ms 控制流程完整跑过之后才刷新 IWDG */
  const uint32_t main_task_cnt = g_main_task_alive_cnt;
  if (main_task_cnt != g_last_main_task_cnt) {
    g_last_main_task_cnt = main_task_cnt;
    HAL_IWDG_Refresh(&hiwdg);
  }

  /* 4. 刷新调试快照（只写全局结构体，不参与控制） */
  debug::Update(g_dbus_rx_buf);
}

void NotifyMainTaskAlive(void)
{
  g_main_task_alive_cnt = g_main_task_alive_cnt + 1u;
}

/**
 * @brief   UART 接收事件回调（空闲 / 满传输）
 * @param   huart: 串口句柄
 * @param   Size: 本次接收到的字节数
 * @retval  None
 * @note    收满 18 字节才认为是完整 DBUS 帧；无论是否解包成功都要重启接收。
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
