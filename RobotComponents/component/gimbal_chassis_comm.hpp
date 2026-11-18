/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm.hpp
 * @brief     : 底盘 ⇄ 云台 双板 CAN 通讯协议
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 完成字段设计与编解码
 *******************************************************************************
 * @attention :
 *  1. 双板只能通过本类的 8 字节 CAN 标准帧交换信息，严禁跨板调用函数或共享变量。
 *  2. CAN ID 约定（两板必须一致）：
 *       C2G（底盘 → 云台）= 0x210
 *       G2C（云台 → 底盘）= 0x211
 *     已避开电机帧：M3508 控制 0x200 / 0x1FF、反馈 0x201~0x208；
 *                   DM-J4310 发送 0x01~0x0F、反馈 0x11~0x1F。
 *  3. 报文布局（两个方向共用同一套布局，保证 encode / decode 严格对称）：
 *       byte 0 : 标志位  b0 = 就绪  b1 = 急停/保护  b2~b3 = 工作模式
 *       byte 1 : 生命状态（0 Dead / 1 Resurrection / 2 Working）
 *       byte 2~3: angle0，int16 大端，比例 1/5000 rad（底盘帧：底盘 IMU yaw；
 *                 云台帧：云台 yaw 关节角 δ）
 *       byte 4~5: angle1，int16 大端，比例 1/5000 rad（底盘帧：期望云台 yaw 目标；
 *                 云台帧：云台 pitch 实际角）
 *       byte 6 : extra，int8（底盘帧：期望云台 pitch 目标 ×100；云台帧：yaw 角速度 ×10）
 *       byte 7 : 校验和 = byte0~byte6 的按字节累加（uint8）
 *  4. 比例系数 kAngleScale 收发两端必须完全一致，否则会出现「角度对不上」的错乱数据。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_CHASSIS_COMM_HPP_
#define GIMBAL_CHASSIS_COMM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "module_state.hpp"
#include "tick.hpp"

/* Exported macro ------------------------------------------------------------*/
namespace robot {
/* Exported constants --------------------------------------------------------*/

/* 双板通讯帧长（定长 8 字节） */
static constexpr uint8_t kCommFrameLen = 8u;

/* Exported types ------------------------------------------------------------*/

/* 底盘↔云台双板 CAN 通讯协议类。底盘板与云台板各实例化一个，用 code_part 区分方向。 */
class GimbalChassisComm {
 public:
  /* 标记本实例运行在哪块板，决定 encode / decode 的方向。 */
  enum class CodePart : uint8_t { Chassis = 0, Gimbal = 1 };

  /* 推荐 CAN ID：C2G = 0x210，G2C = 0x211 */
  static constexpr uint32_t kChassisTxId = 0x210u;
  static constexpr uint32_t kGimbalTxId = 0x211u;

  /* 对方离线阈值，单位：ms（超过该时长未收到新帧即判离线） */
  static constexpr uint32_t kOfflineThresMs = 100u;

  /* 定点化比例系数（收发两端必须一致） */
  static constexpr float kAngleScale = 5000.0f;  ///< rad  → int16
  static constexpr float kPitchScale = 100.0f;   ///< rad  → int8
  static constexpr float kRateScale = 10.0f;     ///< rad/s→ int8
  static constexpr float kAimScale = 127.0f;     ///< [-1,1] → int8（遥控瞄准输入）

  /* 底盘→云台（C2G）帧数据 */
  struct ChassisData {
    bool is_ready = false;               ///< 底盘是否就绪（自检完成且处于 Working）
    bool is_emergency = false;           ///< 底盘是否处于急停 / 保护
    LifeState life_state = LifeState::kDead;   ///< 底盘生命状态
    ChassisMode mode = ChassisMode::kSeparate; ///< 底盘当前工作模式
    float target_gimbal_yaw = 0.0f;      ///< 期望的云台 yaw 目标，单位：rad
    float target_gimbal_pitch = 0.0f;    ///< 期望的云台 pitch 目标，单位：rad
    /* 遥控瞄准输入：**接收机只接在底盘板**时，云台从这里取右摇杆代替本地 DBUS。
     * 云台侧据此实现「本地优先、否则取对板转发」的瞄准输入。*/
    float aim_yaw = 0.0f;                ///< 右摇杆水平原始值 [-1,1]
    float aim_pitch = 0.0f;              ///< 右摇杆竖直原始值 [-1,1]
  };

  /* 云台→底盘（G2C）帧数据 */
  struct GimbalData {
    bool is_ready = false;                  ///< 云台是否就绪
    bool is_emergency = false;              ///< 云台是否处于急停 / 保护
    LifeState life_state = LifeState::kDead;///< 云台生命状态
    GimbalMode mode = GimbalMode::kJoint;   ///< 云台当前工作模式
    float yaw = 0.0f;       ///< 云台 yaw 关节角（= 云台相对底盘偏航角 δ），单位：rad
    float pitch = 0.0f;     ///< 云台 pitch 关节角，单位：rad
    float yaw_rate = 0.0f;  ///< 云台 yaw 角速度，单位：rad/s（可用于前馈，可忽略）
  };

  GimbalChassisComm(CodePart code_part, uint32_t tx_id, uint32_t rx_id);

  /* 编码己方帧：把本地 ChassisData / GimbalData 打包进 8 字节。 */
  void encode(uint8_t tx_data[kCommFrameLen]);
  /* 解码对方帧：把 8 字节还原成对方语义并更新本地缓存。 */
  void decode(const uint8_t rx_data[kCommFrameLen]);

  /* 判断对方是否离线（超过阈值时长未收到新帧）。 */
  bool isOffline() const;

  /* 通讯帧配置访问（CAN 接收分发需要按 rx_id 路由） */
  uint32_t tx_id() const { return tx_id_; }
  uint32_t rx_id() const { return rx_id_; }
  CodePart code_part() const { return code_part_; }

  /* 是否已经成功收到过至少一帧对方数据（用于「从未上线」与「掉线」的区分） */
  bool hasReceivedOnce() const { return has_rx_once_; }

  /* 数据访问（供 RobotModules / Task 读写）。 */
  ChassisData &chassis_data() { return chassis_data_; }
  GimbalData &gimbal_data() { return gimbal_data_; }
  const ChassisData &chassis_data() const { return chassis_data_; }
  const GimbalData &gimbal_data() const { return gimbal_data_; }

 private:
  /* 计算 8 字节帧的累加校验和 */
  static uint8_t CalcChecksum(const uint8_t data[kCommFrameLen]);

  CodePart code_part_;
  uint32_t tx_id_;
  uint32_t rx_id_;
  uint32_t last_rx_tick_;  ///< 上次收到对方帧的 GetTickMs() 时间戳
  bool has_rx_once_ = false;  ///< 是否收到过对方帧
  ChassisData chassis_data_;
  GimbalData gimbal_data_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot

#endif /* GIMBAL_CHASSIS_COMM_HPP_ */
