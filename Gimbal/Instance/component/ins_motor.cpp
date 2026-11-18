/**
 *******************************************************************************
 * @file      :ins_motor.cpp
 * @brief     : 云台 yaw / pitch 两个 DM-J4310 关节电机实例化
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 完成关节电机实例化
 *******************************************************************************
 * @attention :
 *  1. 配置依据考核说明 6.2「电机配置」表（以实机接线为准）：
 *       云台 yaw   : DM-J4310，CAN id = 2，反馈 id = 0x12，方向 正向，角度偏移 -1.36920
 *       云台 pitch : DM-J4310，CAN id = 1，反馈 id = 0x11，方向 反向，角度偏移  0.02480
 *     yaw / pitch 都支持连续旋转 / 软件限位（pitch 的限位在 robot::Gimbal 里做）。
 *  2. 达妙电机**上电默认失能，且不发指令就不回反馈**，所以 auto_enable = true，
 *     由设备类在收到反馈时自动补使能帧。
 *  3. 角度范围必须限制成 [-π, π)：DM-J4310 的位置反馈是单圈绝对角，默认的无限角范围
 *     会累计圈数，把正常机械角误判成数倍 2π 的大误差。
 *  4. 电机对象用**函数内静态**构造（首次 Get 时才初始化），避免 main() 之前跑构造。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_motor.hpp"

#include "motor_DaMiao.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

namespace {

using hello_world::motor::AngleRange;
using hello_world::motor::DM_J4310;
using hello_world::motor::InputType;
using hello_world::motor::OptionalParams;

/* 关节电机 CAN id（与考核说明 6.2 一致；反馈 id 由 DaMiao::GetRxId 得到 id + 0x10） */
constexpr uint8_t kYawMotorId = 2u;
constexpr uint8_t kPitchMotorId = 1u;

/* 6.2 实车标定的角度偏移，单位：rad（使 yaw=0 枪口正对底盘前向、pitch=0 枪口水平） */
/* ⚠ 2026-11-18 实车二次标定：yaw 零点整体平移 +π。
 *   依据（两条独立的实车现象，都能被「上报的关节角比真实值多 π」唯一解释）：
 *     1) 分离模式平移整体取负：推前进车后退、推左车右移  → R(π) = −I；
 *     2) 跟随模式车头最终停在枪口**反方向**：跟随环把 δ 收敛到 0，
 *        若 δ 比真实值多 π，则真实夹角收敛到 −π。
 *   而轮子方向表与麦轮逆解本身是正确的 —— 云台未 Working（δ 被强制置 0）时
 *   前进方向是对的，说明问题不在底盘侧，只在 δ 的零点。
 *   若以后更换 yaw 电机 / 重装方向，先把这里改回 -1.36920f 再重新标定。 */
constexpr float kYawAngleOffset = -1.36920f + 3.14159265f;  /* = +1.77239265 */
constexpr float kPitchAngleOffset = 0.02480f;

/**
 * @brief       生成关节电机的可选配置
 * @param       dir_reversed: 电机正方向与约定正方向是否相反
 * @param       angle_offset: 角度偏移，单位 rad
 * @retval      可选配置
 * @note        输入类型用 kTorq：位置环的输出本来就是力矩（N·m），
 *              走 setInput(torq) 纯力矩透传，控制权完全在自己手里，
 *              不用 MIT 模式（电机内部位置环会和我们的 PID 打架）。
 */
OptionalParams MakeJointParams(bool dir_reversed, float angle_offset)
{
  OptionalParams opt;
  opt.input_type = InputType::kTorq;
  opt.dir = dir_reversed ? hello_world::motor::kDirRev
                         : hello_world::motor::kDirFwd;
  opt.angle_range = AngleRange::kNegPiToPosPi;
  opt.angle_offset = angle_offset;
  return opt;
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/
hello_world::motor::Motor *GetMotorYaw()
{
  /* 6.2：yaw 正向 / 偏移 -1.36920 */
  static DM_J4310 motor(kYawMotorId, MakeJointParams(false, kYawAngleOffset),
                        /*auto_enable=*/true, /*enable_mode_switch=*/false);
  return &motor;
}

hello_world::motor::Motor *GetMotorPitch()
{
  /* 6.2：pitch 反向 / 偏移 0.02480 */
  static DM_J4310 motor(kPitchMotorId, MakeJointParams(true, kPitchAngleOffset),
                        /*auto_enable=*/true, /*enable_mode_switch=*/false);
  return &motor;
}

/* Private function definitions ----------------------------------------------*/
