/**
 *******************************************************************************
 * @file      :robot.hpp
 * @brief     : 底盘板顶层状态机（Dead / Resurrection / Working）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 完成编写
 *******************************************************************************
 * @attention :
 *  1. 本类是底盘板 1ms 控制周期的唯一入口，按「更新数据 → 安全判定 → 状态机迁移 →
 *     生成指令 → 下发」的顺序组织；任何分支都先过安全判定。
 *  2. 安全红线（考核说明 1.3）在本类中的落地方式：
 *       - 急停 / 遥控器离线 / 双板通讯离线 / 未进入 Working
 *             → 所有电机立刻 setInput(0)（每一拍都下发 0，不依赖上一条指令残留）
 *       - 只有「有效 DBUS + 右拨杆处于运行档 + 左拨杆选定模式」三件事同时成立，
 *         才允许输出非零指令
 *       - 小陀螺有明确进出档位（左拨杆下）且角速度限幅（Chassis::kMaxSpinOmega）
 *       - 代码中没有任何无条件循环、常值 / 递增电流等不受遥控器约束的逻辑
 *  3. 遥控器档位约定（见 namespace 内常量，可统一修改）：
 *       右拨杆：上 = 运行（唯一允许出力的档位）
 *               中 = 待机（零输出）
 *               下 = 急停（立刻归零并回 Dead）
 *       左拨杆：上 = 分离   中 = 跟随   下 = 小陀螺
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_ROBOT_MODULES_ROBOT_HPP_
#define CHASSIS_ROBOT_MODULES_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "DT7.hpp"
#include "module_state.hpp"

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

namespace robot {

/* 前向声明，避免头文件相互包含 */
class Chassis;
class GimbalChassisComm;

}  // namespace robot

namespace hello_world {
namespace imu {
class Imu;
}  // namespace imu
}  // namespace hello_world

namespace robot {

/* 底盘板顶层状态机 */
class Robot {
 public:
  /* ---------------- 遥控器档位约定（统一在此修改） ---------------- */
  static constexpr uint8_t kRightSwitchRun =
      static_cast<uint8_t>(hello_world::remote_control::kSwitchStateUp);    ///< 运行
  static constexpr uint8_t kRightSwitchEmergency =
      static_cast<uint8_t>(hello_world::remote_control::kSwitchStateDown);  ///< 急停

  static constexpr uint8_t kLeftSwitchSeparate =
      static_cast<uint8_t>(hello_world::remote_control::kSwitchStateUp);   ///< 分离
  static constexpr uint8_t kLeftSwitchFollow =
      static_cast<uint8_t>(hello_world::remote_control::kSwitchStateMid);  ///< 跟随
  static constexpr uint8_t kLeftSwitchSpin =
      static_cast<uint8_t>(hello_world::remote_control::kSwitchStateDown); ///< 小陀螺

  /* ---------------- 控制参数 ---------------- */
  static constexpr float kLoopDt = 0.001f;            ///< 控制周期，单位：s（TIM6 = 1ms）
  static constexpr uint32_t kResurrectionHoldMs = 300;///< 恢复自检保持时间，单位：ms
  /* 摇杆 / 拨轮死区。实测 DT7 摇杆回中后仍可能残留 ±0.05 抖动，
   * 死区太小会让目标速度在正负之间跳，表现为轮子前后交替滚动。*/
  static constexpr float kStickDeadzone = 0.10f;
  static constexpr float kSpinDeadzone = 0.10f;       ///< 小陀螺拨轮死区
  /* 档位去抖周期数：左拨杆连续 N 个周期读到同一个档位才真正切换模式。
   * 拨杆接触不良、或个别 DBUS 帧误码时档位会瞬变；不去抖的话会瞬间误入小陀螺档，
   * 表现为「偶发的转向全反」。30 ms 对正常换档毫无影响。
   * ⚠ 右拨杆（运行/急停）**不做去抖**：急停必须当拍生效。 */
  static constexpr uint8_t kModeDebounceCycles = 30;

  /* ---------------- 安全开关（验收时必须全部为 1） ----------------
   * 用宏而不是直接用 constexpr，是为了能在编译期用 #warning 提醒。
   * 工程内部一律使用 kRequireImuReady / kRequireGimbalOnline 两个常量。 */
#define ROBOT_REQUIRE_IMU_READY 1
  /* 双板互保：云台板在线才允许出力。
   * ⚠ 当前置 0：云台板已擦空，单人调试底盘。（验收前必须改回 1） */
#define ROBOT_REQUIRE_GIMBAL_ONLINE 1

#if (ROBOT_REQUIRE_IMU_READY == 0)
#warning "ROBOT_REQUIRE_IMU_READY=0: IMU 零漂判据已关闭（仅台架调试）。验收前必须改回 1。"
#endif
#if (ROBOT_REQUIRE_GIMBAL_ONLINE == 0)
#warning "ROBOT_REQUIRE_GIMBAL_ONLINE=0: 双板互保已关闭（云台板擦空/单板调试）。验收前必须改回 1。"
#endif

  ///< 是否要求 IMU 零漂收敛后才允许进入 Working（考核说明 3.4 硬性要求）
  static constexpr bool kRequireImuReady = (ROBOT_REQUIRE_IMU_READY != 0);
  ///< 是否要求双板通讯在线后才允许输出（双板互保；单板台架调试可临时置 0）
  static constexpr bool kRequireGimbalOnline = (ROBOT_REQUIRE_GIMBAL_ONLINE != 0);

  /* 跟随模式是否要求云台处于「绝对朝向」模式（G2C 的 kCartesian）。
   *
   * 为什么必须有这条：跟随环的控制量是「云台相对底盘的关节角 δ」。若云台只做
   * 关节空间位置闭环，底盘一转云台就跟着转，δ 保持常数 → 跟随环持续输出同一个 w
   * → 底盘匀速自转停不下来，等于失控持续转动（1.3 红线）。
   * 因此云台未声明 kCartesian 时，跟随模式只保留云台系平移，不旋转（安全降级）。
   * 云台板实现绝对角控制后，把 G2C 的 mode 位置 1 即可自动启用。
   * ⚠ 只有在确认云台是绝对角控制、且已验证跟随时，才允许临时置 false。 */
  static constexpr bool kFollowRequireAbsoluteGimbal = true;

  Robot(void) = default;

  /* ---------------- 模块注册（由 MainTaskInit 按依赖顺序调用） ---------------- */

  ///< 注册底盘运动模块
  void registerChassis(Chassis *chassis) { chassis_ = chassis; }
  ///< 注册本板 IMU
  void registerImu(hello_world::imu::Imu *imu) { imu_ = imu; }
  ///< 注册遥控器解码对象
  void registerRemoteControl(hello_world::remote_control::DT7 *rc) { rc_ = rc; }
  ///< 注册双板通讯对象
  void registerGimbalChassisComm(GimbalChassisComm *comm) { comm_ = comm; }

  /* ---------------- 周期入口 ---------------- */

  /**
   * @brief       1ms 控制周期入口
   * @retval      None
   * @note        本函数自己不开中断、不死等延时，只做一次完整的状态机推进
   */
  void update(void);

  /* ---------------- 状态查询（调试 / 双板通讯用） ---------------- */

  LifeState lifeState(void) const { return life_state_; }
  ChassisMode chassisMode(void) const { return chassis_mode_; }
  ///< 是否处于急停 / 离线等保护状态
  bool isEmergency(void) const { return is_emergency_; }
  ///< 遥控器是否在线
  bool isRcOnline(void) const { return is_rc_online_; }
  ///< 遥控器右拨杆是否在运行档
  bool isManualControlEnabled(void) const { return is_manual_enabled_; }
  ///< 四轮编码器反馈是否都在线（掉线时禁止出力）
  bool isWheelFeedbackOk(void) const { return is_wheel_feedback_ok_; }
  ///< 双板通讯（云台）是否在线
  bool isGimbalOnline(void) const { return is_gimbal_online_; }
  ///< IMU 零漂是否收敛
  bool isImuReady(void) const { return is_imu_ready_; }
  ///< 云台是否处于绝对朝向模式（跟随环可用的前提）
  bool isGimbalAbsolute(void) const { return is_gimbal_absolute_; }
  ///< 云台是否已就绪（δ 可信的前提）
  bool isGimbalReady(void) const { return is_gimbal_ready_; }
  ///< 云台相对底盘偏航角 δ，单位：rad
  float delta(void) const { return delta_; }
  ///< 本周期是否真正输出了非零指令（调试用）
  bool isOutputEnabled(void) const { return is_output_enabled_; }

 private:
  /* ---------------- 状态机内部实现 ---------------- */

  ///< 更新数据：IMU 采样、遥控器在线 / 档位、双板在线、δ
  void updateData(void);
  ///< 安全判定：急停 / 遥控器离线 / 双板离线 / IMU 未就绪
  void updateSafety(void);
  ///< 状态机迁移
  void updateStateMachine(void);
  ///< 生成并下发底盘指令（仅在允许输出时调用）
  void generateCommand(void);
  ///< 失效归零：所有电机立刻 setInput(0)
  void zeroOutput(void);
  ///< 复位控制器残留（状态切换时调用，防电机突变）
  void resetControllers(void);
  ///< 迁移到 Dead
  void enterDead(void);
  ///< 把本板状态写进 C2G 帧
  void updateCommData(void);

  ///< 是否满足「可以输出」的全部条件
  bool isOutputAllowed(void) const;
  ///< 是否满足「可以进入 Working」的全部条件
  bool isWorkingConditionMet(void) const;

  /* ---------------- 注册的模块 ---------------- */
  Chassis *chassis_ = nullptr;               ///< 底盘运动模块
  hello_world::imu::Imu *imu_ = nullptr;     ///< IMU
  hello_world::remote_control::DT7 *rc_ = nullptr;  ///< 遥控器
  GimbalChassisComm *comm_ = nullptr;        ///< 双板通讯

  /* ---------------- 状态 ---------------- */
  LifeState life_state_ = LifeState::kDead;  ///< 生命状态
  ChassisMode chassis_mode_ = ChassisMode::kSeparate;  ///< 底盘工作模式
  uint32_t resurrection_tick_ = 0;           ///< 进入 Resurrection 的时刻，单位：ms

  bool is_emergency_ = false;        ///< 急停 / 保护标志
  bool is_rc_online_ = false;        ///< 遥控器在线
  bool is_manual_enabled_ = false;   ///< 右拨杆在运行档（Up）
  bool is_wheel_feedback_ok_ = false;///< 四轮编码器反馈是否都在线
  bool is_gimbal_online_ = false;    ///< 双板通讯在线
  bool is_gimbal_absolute_ = false;  ///< 云台是否处于绝对朝向模式
  bool is_gimbal_ready_ = false;     ///< 云台是否已就绪（δ 可信）
  uint8_t mode_candidate_ = 0;       ///< 去抖中的档位候选值（左拨杆原始值）
  uint8_t mode_candidate_cnt_ = 0;   ///< 候选值已连续保持的周期数
  bool is_imu_ready_ = false;        ///< IMU 零漂收敛

  /* 跟随环阻尼用：底盘角速度（IMU yaw 差分 + 低通） */
  float chassis_yaw_rate_ = 0.0f;
  float last_imu_yaw_ = 0.0f;      ///< 上一拍 IMU yaw，用于差分
  bool has_last_imu_yaw_ = false;
  bool is_output_enabled_ = false;   ///< 本周期是否输出非零指令
  float delta_ = 0.0f;               ///< 云台相对底盘偏航角 δ，单位：rad
  float gimbal_pitch_ = 0.0f;        ///< 云台 pitch 反馈，单位：rad
};

}  // namespace robot

#endif /* CHASSIS_ROBOT_MODULES_ROBOT_HPP_ */
