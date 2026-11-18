/**
 *******************************************************************************
 * @file      :ins_motor.cpp
 * @brief     : 底盘 4 个 M3508 轮电机实例化
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 完成电机 ID / 方向配置
 *  V1.1.0      2026-11-18      Chassis        2. 按实车标定表修正轮位 / CAN id / 方向，改用电流输入
 *******************************************************************************
 * @attention :
 *  1. 【轮位、CAN id、方向表 —— 实车标定结果，不要凭 id 顺序猜】
 *
 *       索引  轮位         CAN id   反馈 ID   方向
 *       0     FL 左前       1       0x201    反向 kDirRev
 *       1     FR 右前       4       0x204    正向 kDirFwd
 *       2     RL 左后       2       0x202    反向 kDirRev
 *       3     RR 右后       3       0x203    正向 kDirFwd
 *
 *     标定方法：逐个用手转动实体轮子，看哪一帧反馈（0x201~0x204）的转速变化，
 *     即可得到「轮位 → CAN id」；再朝车辆前进方向转，看反馈转速的正负，
 *     负的说明该轮位要 kDirRev，从而得到方向表。
 *     本表与 inc/chassis_ik_solver.hpp 的 WheelIndex 顺序**必须一致**。
 *
 *  2. 【为什么用 kCurr 而不是 kRaw】M3508 走 C620 的电流环模式，控制量在物理上就是
 *     电流（A）。用 kCurr 后 PID 的输出、电流限幅、四轮电流合计限幅都在同一量纲上，
 *     整定和排查都直观；组件内部会按 curr_rat 换算成报文原始值。
 *     量纲换算：±16384 原始值 ↔ ±20 A，即 1 A ≈ 819.2 LSB。
 *
 *  3. 电机对象用**函数内静态**：首次 GetMotorWheel() 时才构造。
 *     调用方（CommTaskInit）必须在**使能 CAN 接收中断之前**预热一次 GetMotorWheel(0..3)，
 *     避免在接收中断里才首次构造。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_motor.hpp"

#include "motor_M3508.hpp"
#include "tick.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types ---------------------------------------------------------*/

namespace {

using hello_world::motor::AngleRange;
using hello_world::motor::Dir;
using hello_world::motor::InputType;
using hello_world::motor::M3508;
using hello_world::motor::OptionalParams;

/* 轮电机数量，与 chassis_ik_solver::kWheelNum 保持一致 */
constexpr int kWheelMotorNum = 4;

/* 每个轮电机的硬件配置（索引顺序 FL / FR / RL / RR，见文件头表） */
struct WheelMotorConfig {
  uint8_t id;        ///< 电机 CAN id（决定 0x200 帧内位置与 0x200+id 反馈 ID）
  Dir direction;     ///< 电机转动正方向与约定正方向的关系
  float angle_offset;///< 电机角度偏移，单位：rad（轮子角度不参与控制，取 0）
};

const WheelMotorConfig kWheelMotorConfigs[kWheelMotorNum] = {
    {1u, hello_world::motor::kDirRev, 0.0f},  ///< FL 左前
    {4u, hello_world::motor::kDirFwd, 0.0f},  ///< FR 右前
    {2u, hello_world::motor::kDirRev, 0.0f},  ///< RL 左后
    {3u, hello_world::motor::kDirFwd, 0.0f},  ///< RR 右后
};

/* 生成单个轮电机的可选配置 */
OptionalParams MakeWheelMotorParams(const WheelMotorConfig &cfg)
{
  OptionalParams opt;
  /* 输入类型：转子电流（A）。PID 输出、限幅、合计限幅都在 A 量纲上 */
  opt.input_type = InputType::kCurr;
  /* 轮子只用到角速度，角度范围取有界的 [-π, π) 即可，避免累计圈数无限增长 */
  opt.angle_range = AngleRange::kNegPiToPosPi;
  opt.dir = cfg.direction;
  opt.angle_offset = cfg.angle_offset;
  return opt;
}

/* 首次调用时完成 4 个轮电机的构造（函数内静态，避免 main() 之前构造） */
M3508 *WheelMotors(void)
{
  static M3508 motors[kWheelMotorNum] = {
      M3508(kWheelMotorConfigs[0].id, MakeWheelMotorParams(kWheelMotorConfigs[0])),
      M3508(kWheelMotorConfigs[1].id, MakeWheelMotorParams(kWheelMotorConfigs[1])),
      M3508(kWheelMotorConfigs[2].id, MakeWheelMotorParams(kWheelMotorConfigs[2])),
      M3508(kWheelMotorConfigs[3].id, MakeWheelMotorParams(kWheelMotorConfigs[3])),
  };
  return motors;
}

/* ---------------- 编码器反馈在线检测 ---------------- */

/* 反馈掉线阈值：C620 反馈周期 1 ms，100 ms 收不到即可判掉线 */
constexpr uint32_t kFeedbackOfflineThresMs = 100u;

/* 最近一次收到反馈的时刻，单位：ms（ISR 里写、1ms 任务里读） */
volatile uint32_t g_last_feedback_ms[kWheelMotorNum] = {0u, 0u, 0u, 0u};

/* 是否曾经收到过反馈（区分「上电还没收到」与「掉线」） */
volatile bool g_feedback_seen[kWheelMotorNum] = {false, false, false, false};

}  // namespace

/* Exported function definitions ---------------------------------------------*/
hello_world::motor::Motor *GetMotorWheel(int idx)
{
  /* 越界保护：返回 nullptr，由调用方判空，避免野指针访问 */
  if (idx < 0 || idx >= kWheelMotorNum) {
    return nullptr;
  }
  return &WheelMotors()[idx];
}

void NotifyWheelFeedback(int idx)
{
  if (idx < 0 || idx >= kWheelMotorNum) {
    return;
  }
  g_last_feedback_ms[idx] = hello_world::tick::GetTickMs();
  g_feedback_seen[idx] = true;
}

bool IsWheelFeedbackOnline(int idx)
{
  if (idx < 0 || idx >= kWheelMotorNum) {
    return false;
  }
  if (!g_feedback_seen[idx]) {
    return false;
  }
  /* uint32 无符号减法天然处理 GetTickMs 回绕 */
  const uint32_t elapsed = hello_world::tick::GetTickMs() - g_last_feedback_ms[idx];
  return elapsed <= kFeedbackOfflineThresMs;
}

/* Private function definitions ----------------------------------------------*/
