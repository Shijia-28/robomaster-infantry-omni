/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm.cpp
 * @brief     : 底盘 ⇄ 云台 双板 CAN 通讯协议实现
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 完成 encode / decode / isOffline
 *******************************************************************************
 * @attention :
 *  报文布局见 gimbal_chassis_comm.hpp 的 attention 注释，两个方向共用同一套布局，
 *  因此 encode() 与 decode() 完全对称：同一份 8 字节先 encode 再 decode 必须能还原原值。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "gimbal_chassis_comm.hpp"

#include <cmath>
#include <cstring>

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

namespace {

/* 标志位定义（byte0） */
constexpr uint8_t kFlagReady = 1u << 0;
constexpr uint8_t kFlagEmergency = 1u << 1;
constexpr uint8_t kModeShift = 2;  ///< b2~b3 存放工作模式

/**
 * @brief       把浮点量定点化为 int32 并饱和到 int16 范围
 * @param        value: 待定点化数值
 * @param        scale: 比例系数（rad → 整数）
 * @retval       定点化结果
 * @note        先饱和再转 int16，避免数值超范围时出现未定义行为
 */
int16_t ToInt16(float value, float scale)
{
  float tmp = roundf(value * scale);
  if (tmp > 32767.0f) {
    tmp = 32767.0f;
  } else if (tmp < -32768.0f) {
    tmp = -32768.0f;
  }
  return static_cast<int16_t>(tmp);
}

/**
 * @brief       把浮点量定点化为 int32 并饱和到 int8 范围
 * @param        value: 待定点化数值
 * @param        scale: 比例系数
 * @retval       定点化结果
 * @note        None
 */
int8_t ToInt8(float value, float scale)
{
  float tmp = roundf(value * scale);
  if (tmp > 127.0f) {
    tmp = 127.0f;
  } else if (tmp < -128.0f) {
    tmp = -128.0f;
  }
  return static_cast<int8_t>(tmp);
}

/**
 * @brief       把 int16 定点数还原为浮点量
 * @param        value: 定点数
 * @param        scale: 比例系数（整数 → rad）
 * @retval       还原后的浮点量
 * @note        None
 */
float FromInt16(int16_t value, float scale) { return static_cast<float>(value) / scale; }

/**
 * @brief       把 int8 定点数还原为浮点量
 * @param        value: 定点数
 * @param        scale: 比例系数
 * @retval       还原后的浮点量
 * @note        None
 */
float FromInt8(int8_t value, float scale) { return static_cast<float>(value) / scale; }

/* 大端写入 int16 */
void WriteInt16BE(uint8_t *buf, int16_t value)
{
  const uint16_t raw = static_cast<uint16_t>(value);
  buf[0] = static_cast<uint8_t>((raw >> 8) & 0xFFu);
  buf[1] = static_cast<uint8_t>(raw & 0xFFu);
}

/* 大端读取 int16 */
int16_t ReadInt16BE(const uint8_t *buf)
{
  const uint16_t raw = static_cast<uint16_t>((static_cast<uint16_t>(buf[0]) << 8) |
                                             static_cast<uint16_t>(buf[1]));
  return static_cast<int16_t>(raw);
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/
robot::GimbalChassisComm::GimbalChassisComm(robot::GimbalChassisComm::CodePart code_part,
                                            uint32_t tx_id, uint32_t rx_id)
    : code_part_(code_part),
      tx_id_(tx_id),
      rx_id_(rx_id),
      last_rx_tick_(hello_world::tick::GetTickMs()),
      has_rx_once_(false)
{
}

void robot::GimbalChassisComm::encode(uint8_t tx_data[kCommFrameLen])
{
  if (tx_data == nullptr) {
    return;
  }

  /* 统一先清零，保证未使用的位一定是 0（便于对面判错） */
  memset(tx_data, 0, kCommFrameLen);

  uint8_t flags = 0u;
  int16_t angle0 = 0;
  int16_t angle1 = 0;
  int8_t extra = 0;

  if (code_part_ == CodePart::Chassis) {
    /* 本板是底盘：打包 ChassisData（C2G） */
    if (chassis_data_.is_ready) {
      flags |= kFlagReady;
    }
    if (chassis_data_.is_emergency) {
      flags |= kFlagEmergency;
    }
    flags |= static_cast<uint8_t>(
        (static_cast<uint8_t>(chassis_data_.mode) & 0x03u) << kModeShift);

    tx_data[1] = static_cast<uint8_t>(chassis_data_.life_state);
    /* C2G 的 byte2/byte3 用来转发遥控摇杆（云台板没有本地 DBUS 时靠它瞄准）。
     * ⚠ 这两个字节**必须在下面**统一写角度之后才能覆盖，见本函数末尾。 */
    angle1 = ToInt16(chassis_data_.target_gimbal_yaw, kAngleScale);
    extra = ToInt8(chassis_data_.target_gimbal_pitch, kPitchScale);
  } else {
    /* 本板是云台：打包 GimbalData（G2C） */
    if (gimbal_data_.is_ready) {
      flags |= kFlagReady;
    }
    if (gimbal_data_.is_emergency) {
      flags |= kFlagEmergency;
    }
    flags |= static_cast<uint8_t>(
        (static_cast<uint8_t>(gimbal_data_.mode) & 0x03u) << kModeShift);

    tx_data[1] = static_cast<uint8_t>(gimbal_data_.life_state);
    angle0 = ToInt16(gimbal_data_.yaw, kAngleScale);
    angle1 = ToInt16(gimbal_data_.pitch, kAngleScale);
    extra = ToInt8(gimbal_data_.yaw_rate, kRateScale);
  }

  tx_data[0] = flags;
  WriteInt16BE(&tx_data[2], angle0);
  WriteInt16BE(&tx_data[4], angle1);
  tx_data[6] = static_cast<uint8_t>(extra);

  /* 底盘 → 云台：把 byte2/byte3 覆盖成**转发的右摇杆**。
   * ⚠ 必须放在上面 WriteInt16BE(&tx_data[2], angle0) **之后**。
   *   之前写在这句话前面，被 angle0（底盘分支恒为 0）覆盖掉了，
   *   结果云台收到的 aim 永远是 0 —— 表现为「云台很紧、但推右摇杆完全不动」。
   *   校验和必须最后算，否则校验的是被覆盖前的字节。 */
  if (code_part_ == CodePart::Chassis) {
    tx_data[2] = static_cast<uint8_t>(ToInt8(chassis_data_.aim_yaw, kAimScale));
    tx_data[3] = static_cast<uint8_t>(ToInt8(chassis_data_.aim_pitch, kAimScale));
  }

  tx_data[7] = CalcChecksum(tx_data);
}

void robot::GimbalChassisComm::decode(const uint8_t rx_data[kCommFrameLen])
{
  if (rx_data == nullptr) {
    return;
  }

  /* 校验和不通过：整帧丢弃，且不刷新在线时间戳（防止错帧被当作有效数据） */
  if (rx_data[7] != CalcChecksum(rx_data)) {
    return;
  }

  const uint8_t flags = rx_data[0];
  const bool is_ready = (flags & kFlagReady) != 0u;
  const bool is_emergency = (flags & kFlagEmergency) != 0u;
  const uint8_t mode_raw = static_cast<uint8_t>((flags >> kModeShift) & 0x03u);
  LifeState life_state = LifeState::kDead;
  if (rx_data[1] <= static_cast<uint8_t>(LifeState::kWorking)) {
    life_state = static_cast<LifeState>(rx_data[1]);
  }

  const int16_t angle0_raw = ReadInt16BE(&rx_data[2]);
  const int16_t angle1_raw = ReadInt16BE(&rx_data[4]);
  const int8_t extra_raw = static_cast<int8_t>(rx_data[6]);

  if (code_part_ == CodePart::Chassis) {
    /* 本板是底盘：收到的是 G2C，还原 GimbalData */
    gimbal_data_.is_ready = is_ready;
    gimbal_data_.is_emergency = is_emergency;
    gimbal_data_.life_state = life_state;
    /* 云台模式：0 = 关节空间，1 = 笛卡尔（绝对朝向）；非法值按最保守的 kJoint 处理 */
    gimbal_data_.mode = (mode_raw == static_cast<uint8_t>(GimbalMode::kCartesian))
                            ? GimbalMode::kCartesian
                            : GimbalMode::kJoint;
    gimbal_data_.yaw = FromInt16(angle0_raw, kAngleScale);
    gimbal_data_.pitch = FromInt16(angle1_raw, kAngleScale);
    gimbal_data_.yaw_rate = FromInt8(extra_raw, kRateScale);
  } else {
    /* 本板是云台：收到的是 C2G，还原 ChassisData */
    chassis_data_.is_ready = is_ready;
    chassis_data_.is_emergency = is_emergency;
    chassis_data_.life_state = life_state;
    switch (mode_raw) {
      case static_cast<uint8_t>(ChassisMode::kFollow):
        chassis_data_.mode = ChassisMode::kFollow;
        break;
      case static_cast<uint8_t>(ChassisMode::kSpin):
        chassis_data_.mode = ChassisMode::kSpin;
        break;
      default:
        chassis_data_.mode = ChassisMode::kSeparate;
        break;
    }
    chassis_data_.target_gimbal_yaw = FromInt16(angle1_raw, kAngleScale);
    chassis_data_.target_gimbal_pitch = FromInt8(extra_raw, kPitchScale);
    /* byte2/byte3 = 底盘转发的右摇杆（瞄准输入） */
    chassis_data_.aim_yaw =
        FromInt8(static_cast<int8_t>(rx_data[2]), kAimScale);
    chassis_data_.aim_pitch =
        FromInt8(static_cast<int8_t>(rx_data[3]), kAimScale);
  }

  /* 只有校验通过的有效帧才刷新在线时间戳 */
  last_rx_tick_ = hello_world::tick::GetTickMs();
  has_rx_once_ = true;
}

bool robot::GimbalChassisComm::isOffline() const
{
  /* 上电后从未收到过对方的帧 → 视为离线 */
  if (!has_rx_once_) {
    return true;
  }

  const uint32_t now = hello_world::tick::GetTickMs();
  /* uint32 无符号减法天然处理 GetTickMs 回绕 */
  const uint32_t elapsed = now - last_rx_tick_;
  return elapsed > kOfflineThresMs;
}

/* Private function definitions ----------------------------------------------*/
uint8_t robot::GimbalChassisComm::CalcChecksum(const uint8_t data[kCommFrameLen])
{
  uint8_t sum = 0u;
  for (uint8_t i = 0u; i < (kCommFrameLen - 1u); ++i) {
    sum = static_cast<uint8_t>(sum + data[i]);
  }
  return sum;
}
