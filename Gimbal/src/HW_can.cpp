/**
 *******************************************************************************
 * @file      : HW_can.cpp
 * @brief     : CAN 收发基础封装实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-09-18      Hello World     1. 考核骨架：CanFilter_Init + CAN_Send_Msg
 *                                                全量提供，接收分发回调掏空
 *******************************************************************************
 * @attention :
 *  接收中断回调 HAL_CAN_RxFifo0/1MsgPendingCallback 已在此预留，仅完成「取 8 字节 +
 *  再次使能接收中断」，具体「按帧 ID 分发」逻辑留给新人：
 *    - M3508 / DM 电机反馈：调用对应电机的 decode(can?_rx_data)
 *    - 双板通讯帧：调用新人自写的协议解码
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_can.hpp"

#include "can.h"

#include "gimbal_chassis_comm.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_motor.hpp"
#include "motor_DaMiao.hpp"

/* Private constants ---------------------------------------------------------*/

namespace {

/**
 * @brief       按标准帧 ID 分发一帧 CAN 数据
 * @param       std_id: 帧的标准 ID
 * @param       data: 8 字节数据
 * @retval      None
 * @note        路由规则：
 *               1) 命中某个达妙电机的反馈 ID（id + 0x10，即 0x11 / 0x12）→ 该电机 decode()
 *               2) 命中双板通讯帧 ID（0x210，C2G）→ GimbalChassisComm decode()
 *               3) 都不命中 → 丢弃
 *              CAN1 / CAN2 都调用本函数，所以电机与双板帧接在哪一路总线都能正常收发。
 */
void DispatchCanFrame(uint32_t std_id, const uint8_t data[8])
{
  /* 1. 两轴达妙电机反馈 */
  hello_world::motor::Motor *const motors[2] = {GetMotorYaw(), GetMotorPitch()};
  for (int i = 0; i < 2; ++i) {
    hello_world::motor::Motor *motor = motors[i];
    if (motor == nullptr) {
      continue;
    }
    if (std_id == hello_world::motor::DaMiao::GetRxId(motor->id())) {
      motor->decode(data);
      return;
    }
  }

  /* 2. 双板通讯帧：只有本板配置的接收 ID 才处理 */
  robot::GimbalChassisComm *comm = GetGimbalChassisComm();
  if ((comm != nullptr) && (std_id == comm->rx_id())) {
    comm->decode(data);
    return;
  }

  /* 3. 其余帧（底盘电机反馈、裁判系统等）本板不关心，丢弃 */
}

}  // namespace

/* Private variables ---------------------------------------------------------*/
static CAN_RxHeaderTypeDef rx_header1;
static CAN_RxHeaderTypeDef rx_header2;
static uint8_t can1_rx_data[8];
static uint8_t can2_rx_data[8];
static uint32_t p_tx_mailbox;

/* Exported function definitions ---------------------------------------------*/

void CanFilter_Init(CAN_HandleTypeDef *hcan)
{
  /* ⚠ 必须整体清零：SlaveStartFilterBank 是双 CAN 芯片的必填项，HAL 会把它写进
   *   CAN1->FMR 的 CAN2SB 位。骨架原版用的是未初始化的局部结构体且没赋这个字段，
   *   CAN2SB 会被写成栈上的随机值 —— 一旦 ≠ 14，CAN2 就一个滤波器都没有，
   *   表现为「发得出去、收不回来」。详见 Chassis/src/HW_can.cpp 的同一处注释。 */
  CAN_FilterTypeDef can_filter = {0};

  /* 全收：掩码全 0，任何 ID 均通过 */
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.FilterActivation = ENABLE;

  /* CAN1 用 bank 0~13、CAN2 用 bank 14~27，所以 CAN2 的起始 bank 必须是 14 */
  can_filter.SlaveStartFilterBank = 14;

  if (hcan == &hcan1) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterBank = 0;
  } else if (hcan == &hcan2) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    can_filter.FilterBank = 14;
  } else {
    return;
  }

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }
}

void CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t id,
                  uint8_t len)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  tx_header.StdId = id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = len;
  HAL_CAN_AddTxMessage(hcan, &tx_header, msg, &p_tx_mailbox);
}

/**
 * @brief   CAN1 FIFO0 接收中断回调（新人填写分发逻辑）
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    此处只取了 8 字节并重新使能接收中断，具体分发由新人实现
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1,
                             can1_rx_data) == HAL_OK) {
      /* 只处理标准帧：达妙反馈、双板通讯帧都是标准帧 */
      if (rx_header1.IDE == CAN_ID_STD) {
        DispatchCanFrame(rx_header1.StdId, can1_rx_data);
      }
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief   CAN2 FIFO1 接收中断回调（新人填写分发逻辑）
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    同上，具体分发由新人实现
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2,
                             can2_rx_data) == HAL_OK) {
      /* 同 FIFO0：按标准帧 ID 路由 */
      if (rx_header2.IDE == CAN_ID_STD) {
        DispatchCanFrame(rx_header2.StdId, can2_rx_data);
      }
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}
