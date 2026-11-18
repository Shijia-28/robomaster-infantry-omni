/**
 *******************************************************************************
 * @file      :gimbal.hpp
 * @brief     : 云台运动模块：yaw / pitch 关节位置环（角度 -> 力矩）
 * @history   :
 *  Version     Date            Author          Note
 *  V2.0.0      2026-11-18      Chassis        1. 重构内部结构（行为与 V1 完全等价）
 *******************************************************************************
 * @attention :
 *  1. 本类只做关节空间控制：目标角度 -> PID -> 力矩 -> 电机。
 *     遥控、状态机、安全判据都不在这里（见 robot::Robot）。
 *  2. 依赖注入：电机与 PID 由 init() 传入，所以本文件只前向声明、不含实现头。
 *  3. 【yaw 的连续角】达妙 angle() 是单圈绝对角，在 ±π 处跳变。
 *     直接拿它做误差，跨零那一拍误差会从 -0.01 跳成 +6.27，输出瞬间打满。
 *     本类用 AngleTracker 把角度序列还原成单调连续的角，再把目标展开到最近的
 *     等价角，误差永远是"最短路径"—— 这样 359° -> 1° 只走 +2°。
 *     输出走 setInput(torq) 纯力矩，没有电机内部位置环的 ±π 限制，yaw 可多圈连续转。
 *  4. 【pitch 的软件限位】目标先钳到 [pitch_min, pitch_max]。
 *     限位判据必须用**反馈角度**：被外力推过界时只有反馈才反映真实位置，
 *     误差自然反向，PID 会把机构拉回来。
 *  5. 【绝对角 yaw】开启后 yaw 的反馈量从"关节角"换成"IMU 绝对朝向"，底盘转动时
 *     云台自动补偿，这是底盘跟随模式（kCartesian）的前提。
 *     IMU 未就绪时自动退回关节角，绝不拿未收敛的姿态去闭环。
 *     反馈源切换的那一拍会做无扰对齐（见 updateYawAxis）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_ROBOT_MODULES_GIMBAL_HPP_
#define GIMBAL_ROBOT_MODULES_GIMBAL_HPP_

/* Includes ------------------------------------------------------------------*/
#include "module_state.hpp"

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

namespace hello_world {
namespace motor {
class Motor;
}  // namespace motor
namespace imu {
class Imu;
}  // namespace imu
namespace pid {
class Pid;
}  // namespace pid
}  // namespace hello_world

namespace robot {

/**
 * @brief 云台关节空间控制模块
 * @note  使用顺序：init() 一次 -> setTarget() -> update(dt)（1kHz）
 *        stop() 可随时调用，效果是**卸力**（两轴立刻输出 0 力矩），不是回零
 */
class Gimbal {
 public:
  /* 关节限位与闭环绕配置 */
  struct Config {
    float pitch_min = -0.40f;  ///< rad，向下保守下限（必须按实机机械行程复核）
    float pitch_max = 0.60f;   ///< rad，向上保守上限
    bool absolute_yaw = true;  ///< yaw 是否用 IMU 绝对角闭环
  };

  Gimbal(void) = default;

  /* ---------------- 生命周期与依赖 ---------------- */

  /**
   * @brief       注入依赖并复位内部状态
   * @param       yaw_motor / pitch_motor: 关节电机（输入类型必须是 kTorq）
   * @param       yaw_pid / pitch_pid: 位置环 PID（输出即力矩 N·m）
   * @param       cfg: 限位 / 绝对角配置
   * @retval      None
   * @note        任一电机或 PID 为空时对应轴不输出（便于分轴调试）；
   *              imu_ 由 setImu() 注入，本函数**不清**它。
   */
  void init(hello_world::motor::Motor *yaw_motor,
            hello_world::motor::Motor *pitch_motor,
            hello_world::pid::Pid *yaw_pid,
            hello_world::pid::Pid *pitch_pid, const Config &cfg);

  ///< 注入 IMU（绝对角模式用）；传 nullptr 则永远退回关节角
  void setImu(hello_world::imu::Imu *imu) { imu_ = imu; }

  /**
   * @brief       下发关节目标角度并进入闭环
   * @param       target: yaw / pitch，单位 rad
   * @retval      None
   * @note        急停之后重新给目标即恢复控制（不再需要额外的使能调用）
   */
  void setTarget(const GimbalAngle &target);

  /**
   * @brief       卸力：两轴立刻 setInput(0)，目标跟随当前实际角
   * @retval      None
   * @note        急停 / 离线 / 未进 Working 时必须调用（考核说明 1.3 一票否决项）。
   *              这里必须**直接清电机输入**，不能只置 enabled_=false —— 上层的
   *              急停分支调完 stop() 就 return，永远走不到 update() 里的清零点。
   */
  void stop(void);

  /**
   * @brief       1kHz 周期入口：读角度 -> PID -> setInput
   * @param       dt: 采样周期，单位 s
   * @retval      None
   * @note        None
   */
  void update(float dt);

  /* ---------------- 状态查询 ---------------- */

  ///< 本拍**实际**是否在用绝对角反馈（双板帧上报的模式必须用这个值）
  bool absoluteActive(void) const { return last_absolute_; }
  ///< 本拍 yaw 环实际使用的反馈量（关节角或绝对角，单位 rad）
  float yawFeedback(void) const { return last_feedback_yaw_; }
  ///< yaw 的连续角（跨 ±π 不跳变），仅供调试观察
  float yawContinuous(void) const { return yaw_cont_; }
  ///< 上一拍实际下发的力矩，单位 N·m（仅供调试观察）
  float yawOutput(void) const { return yaw_out_; }
  float pitchOutput(void) const { return pitch_out_; }

  ///< 是否已到位（yaw 用最短路径差，pitch 用直接差）
  bool atTarget(float tol = 0.02f) const;

  const GimbalAngle &target(void) const { return target_; }
  const GimbalAngle &actual(void) const { return actual_; }
  bool enabled(void) const { return enabled_; }

  /**
   * @brief       自上一拍以来 yaw 反馈源是否切换过（关节角 ⇄ 绝对角）
   * @retval      true = 刚切换，调用方应把目标角对齐到 yawFeedback()
   * @note        一次性标志：读到即清除
   */
  bool consumeFeedbackSwitch(void)
  {
    const bool v = feedback_switched_;
    feedback_switched_ = false;
    return v;
  }

 private:
  /* ---------------- 内部实现 ---------------- */

  /**
   * @brief  连续角跟踪器
   * @note   把"单圈绝对角"序列还原成单调连续角：每拍只累加**相邻两拍的差值**
   *         （差值先归一到 [-π,π]），所以跨 ±π 时不会跳 2π。
   *         invalidate() 用于反馈源切换：两种量纲的差不是真实转动，必须重新起算。
   */
  class AngleTracker {
   public:
    ///< 重新起算：下一拍 push() 直接返回输入角
    void invalidate(void) { primed_ = false; }
    ///< 强制把连续角置成某个值
    void force(float angle)
    {
      value_ = angle;
      last_ = angle;
      primed_ = true;
    }
    ///< 输入本拍原始角，返回本拍连续角
    float push(float angle);
    float value(void) const { return value_; }

   private:
    float value_ = 0.0f;  ///< 连续角
    float last_ = 0.0f;   ///< 上一拍的原始角
    bool primed_ = false; ///< 是否已有上一拍
  };

  ///< 未使能时的刷新：只读角度、不发力（从未启用过时连 0 力矩帧都不发）
  void refreshWhenIdle(void);
  ///< yaw 轴：选反馈源 -> 连续角 -> 目标展开 -> PID -> setInput
  void updateYawAxis(float dt);
  ///< pitch 轴：反馈限位 -> PID -> setInput
  void updatePitchAxis(float dt);
  ///< 目标角钳位到 [pitch_min, pitch_max]
  float limitPitch(float angle) const;
  ///< 轴驱动：误差 = target - measure，返回 PID 输出（无 PID 时为 0）
  float driveAxis(hello_world::pid::Pid *pid, float target, float measure,
                  float dt) const;

  hello_world::motor::Motor *yaw_motor_ = nullptr;
  hello_world::motor::Motor *pitch_motor_ = nullptr;
  hello_world::pid::Pid *yaw_pid_ = nullptr;
  hello_world::pid::Pid *pitch_pid_ = nullptr;
  hello_world::imu::Imu *imu_ = nullptr;

  Config cfg_{};
  GimbalAngle target_{};
  GimbalAngle actual_{};

  bool enabled_ = false;  ///< 是否在闭环控制中
  bool started_ = false;  ///< 是否曾经启用过（区分"刚开机"与"急停后卸力"）

  AngleTracker yaw_tracker_{};  ///< yaw 连续角跟踪
  float yaw_cont_ = 0.0f;       ///< 本拍 yaw 连续角（= yaw_tracker_.value()）

  bool last_absolute_ = false;      ///< 上一拍是否真的用了绝对角
  float last_feedback_yaw_ = 0.0f;  ///< 上一拍 yaw 环实际用的反馈量
  bool feedback_switched_ = false;  ///< 反馈源刚切换（一次性，供上层对齐目标）

  float yaw_out_ = 0.0f;    ///< 上一拍 yaw 力矩，N·m（只写不参与控制）
  float pitch_out_ = 0.0f;  ///< 上一拍 pitch 力矩，N·m
};

}  // namespace robot

#endif /* GIMBAL_ROBOT_MODULES_GIMBAL_HPP_ */
