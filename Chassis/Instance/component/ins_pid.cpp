/**
 *******************************************************************************
 * @file      :ins_pid.cpp
 * @brief     : 轮电机速度环 PID 实例化与参数整定
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 完成 4 个轮速环 PID 实例化
 *  V1.1.0      2026-11-18      Chassis        2. 输入类型改为电流（A），参数改到 A 量纲
 *******************************************************************************
 * @attention :
 *  1. 轮速环的物理量（与 ins_motor.cpp 的 input_type = kCurr 对应）：
 *       输入（误差）  轮速误差，单位：rad/s（电机输出端，已扣除减速比）
 *       输出          电流，单位：A（组件内部按 1A ≈ 819.2 LSB 换算成 0x200 报文）
 *  2. 参数说明（起调值，实车必须重新整定）：
 *       被控对象可近似为「电流 → 加速度」的积分环节，因此需要较大的 kp 才能跟得上目标；
 *       ki 用于消除摩擦 / 负载带来的稳态误差；kd 对速度噪声敏感，默认给 0。
 *  3. 整定建议：
 *       先把 ki、kd 置 0，逐渐加大 kp 直到轮速阶跃响应出现轻微超调（10%~20%）；
 *       再加 ki 把稳态误差压到 0，若出现低频振荡则回调 ki 并适当减小 kp。
 *       若目标阶跃瞬间超调仍然偏大，可调用 Pid::set_i_sep_thres() 打开积分分离
 *       （误差大于阈值时暂停积分）。注意阈值必须大于该工况下的稳态误差，否则会
 *       因为积分被永久关掉而留下稳态误差。
 *  4. 限幅取值：
 *       out_limit = 12 A（M3508 连续 20 A 的 60%，留余量给速度环动态）；
 *       i_limit   = 4.5 A（积分单独最多贡献 4.5 A，防止长时间贴限幅后解除时的冲量）。
 *       另有四轮「电流绝对值合计 25 A」的总限幅在 Chassis::update() 里，
 *       两道限幅一起保证不会把电池 / 分电板顶到保护。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_pid.hpp"

#include "pid.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

namespace {

/* 轮速环 PID 数量，与底盘轮数一致 */
constexpr int kWheelPidNum = 4;

/* 轮速环 PID 起调参数（电流量纲）
 *
 * 定标依据（host 侧仿真，空载 M3508 + 6 寸麦轮）：
 *   kp = 0.5 时「正转 4.73 → 反转 4.73」要 1.70 s 才换过来 —— 操作手把拨轮反向之后
 *   轮子还会继续朝原方向转近 2 秒，看起来就是「方向全错」。kp ≥ 1.5 后换向 < 0.11 s。
 *
 * ⚠ Ki 先保持 0（首轮整定只开 P）。积分项是有记忆的：换向前攒下的积分会压过
 *   新指令，表现为「拨轮已经反向、轮子还在往原方向推」；积分给太大还会引起
 *   「松杆后还在推 / 前后摆」。等 P 单独确认稳定、方向都对之后，再按需加 Ki
 *   （建议 Ki ≤ 0.2、积分限幅 ≤ 1.5 A）。 */
constexpr float kWheelPidKp = 1.50f;    ///< 比例系数，单位：A / (rad/s)
constexpr float kWheelPidKi = 0.00f;    ///< 积分系数（第二轮再开）
constexpr float kWheelPidKd = 0.0f;     ///< 微分系数
constexpr float kWheelPidILim = 1.50f;  ///< 积分限幅，单位：A
constexpr float kWheelPidOutLim = 12.0f;///< 输出限幅，单位：A

/* 函数内静态存放 4 个轮速环 PID（避免 main() 之前构造） */
hello_world::pid::Pid *WheelPids(void)
{
  static hello_world::pid::Pid pids[kWheelPidNum] = {
      hello_world::pid::Pid(kWheelPidKp, kWheelPidKi, kWheelPidKd, kWheelPidILim,
                            kWheelPidOutLim),
      hello_world::pid::Pid(kWheelPidKp, kWheelPidKi, kWheelPidKd, kWheelPidILim,
                            kWheelPidOutLim),
      hello_world::pid::Pid(kWheelPidKp, kWheelPidKi, kWheelPidKd, kWheelPidILim,
                            kWheelPidOutLim),
      hello_world::pid::Pid(kWheelPidKp, kWheelPidKi, kWheelPidKd, kWheelPidILim,
                            kWheelPidOutLim),
  };
  return pids;
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/
hello_world::pid::Pid *GetPidMotorWheel(int idx)
{
  if (idx < 0 || idx >= kWheelPidNum) {
    return nullptr;
  }
  return &WheelPids()[idx];
}

/* Private function definitions ----------------------------------------------*/
