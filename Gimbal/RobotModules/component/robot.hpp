/**
 *******************************************************************************
 * @file      :robot.hpp
 * @brief     : 云台板顶层状态机（Dead / Resurrection / Working）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 完成编写
 *******************************************************************************
 * @attention :
 *  1. 本类是云台板 1ms 控制周期的唯一入口，顺序固定为
 *     「更新数据 → 安全判定 → 状态机迁移 → 生成目标角 → 关节控制」，
 *     任何分支都先过安全判定（考核说明 1.3、4.9）。
 *  2. 安全红线在本类中的落地：
 *       - 急停（右拨杆下）/ 遥控器离线 / 未进入 Working → gimbal->stop()，
 *         两轴立刻 setInput(0)；
 *       - 只有「有效 DBUS + 右拨杆在运行档 + 已进 Working」才允许闭环出力；
 *       - 代码里没有任何无条件循环、常值 / 递增力矩。
 *  3. 【摇杆是角速度控制，不是角度映射】右摇杆推住 → 目标角按固定角速度累加，
 *     松手停在当前角度，手感和真实云台一致。若做成「摇杆位置直接映射绝对角」，
 *     小幅推杆就会让云台猛甩。
 *  4. 上电 / 恢复后先锁住当前姿态，摇杆连续回中 50 个周期后才允许积分，
 *     避免上电瞬间的遥控残值把云台拉走。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_ROBOT_MODULES_ROBOT_HPP_
#define GIMBAL_ROBOT_MODULES_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "DT7.hpp"
#include "module_state.hpp"

namespace hello_world {
namespace imu {
class Imu;
}  // namespace imu
}  // namespace hello_world

namespace robot {

class Gimbal;
class GimbalChassisComm;

/* 云台板顶层状态机 */
class Robot {
 public:
  /* 遥控器档位约定（与底盘板一致，统一在此修改） */
  static constexpr uint8_t kRightSwitchRun =
      static_cast<uint8_t>(hello_world::remote_control::kSwitchStateUp);   ///< 运行
  static constexpr uint8_t kRightSwitchEmergency =
      static_cast<uint8_t>(hello_world::remote_control::kSwitchStateDown); ///< 急停

  struct Config {
    /* --- 时序 --- */
    uint32_t dead_dwell_cycles = 200u;  ///< Dead 停留周期数（1kHz 下 200 = 200ms）

    /* --- 摇杆 -> 角速度 --- */
    float max_yaw_rate = 3.0f;    ///< rad/s，右摇杆水平推满时的 yaw 角速度
    float max_pitch_rate = 2.0f;  ///< rad/s
    float stick_deadband = 0.05f; ///< 摇杆死区

    /* --- 目标角钳位（必须与 Gimbal::Config 的限位一致）--- */
    float pitch_min = -0.40f;  ///< rad
    float pitch_max = 0.60f;   ///< rad
    float pitch_home = 0.0f;   ///< rad，进 Working 时的 pitch 初始目标（0 = 枪口水平）

    /* --- 绝对角 yaw（跟随模式的前置）--- */
    bool absolute_yaw = true;  ///< true = IMU 绝对角闭环，G2C 报 kCartesian；否则 kJoint

    /* --- 安全开关（验收时必须全部为 1）--- */
#define GIMBAL_REQUIRE_IMU_READY 1
#define GIMBAL_REQUIRE_REMOTE 1

#if (GIMBAL_REQUIRE_IMU_READY == 0)
#warning "GIMBAL_REQUIRE_IMU_READY=0: IMU 零漂判据已关闭（仅台架调试）。验收前必须改回 1。"
#endif
#if (GIMBAL_REQUIRE_REMOTE == 0)
#warning "GIMBAL_REQUIRE_REMOTE=0: 遥控判据已关闭，违反 1.3 红线。验收前必须改回 1。"
#endif

    ///< 是否要求 IMU 零漂收敛后才允许进入 Working 与绝对角闭环
    bool require_imu_calib = (GIMBAL_REQUIRE_IMU_READY != 0);
    ///< 是否要求有效 DBUS 遥控数据（考核说明 1.3 硬性要求）
    bool require_remote = (GIMBAL_REQUIRE_REMOTE != 0);
  };

  Robot(void) = default;

  /* ---------------- 模块注册（由 MainTaskInit 按依赖顺序调用） ---------------- */

  void init(const Config &cfg);
  void registerGimbal(Gimbal *gimbal);
  void registerImu(hello_world::imu::Imu *imu) { imu_ = imu; }
  void registerRemoteControl(hello_world::remote_control::DT7 *rc) { rc_ = rc; }
  void registerGimbalChassisComm(GimbalChassisComm *comm) { comm_ = comm; }

  /* ---------------- 周期入口 ---------------- */

  /**
   * @brief       1ms 控制周期入口
   * @param       dt: 控制周期，单位 s
   * @retval      None
   * @note        None
   */
  void update(float dt);

  /* ---------------- 状态查询 ---------------- */

  LifeState lifeState(void) const { return life_state_; }
  ///< 上报给底盘的云台模式（**实际生效**的那个，不是配置意图）
  GimbalMode mode(void) const;
  ///< 急停 / 保护中
  bool isEmergency(void) const { return is_emergency_; }
  ///< 遥控器是否在线
  bool isRcOnline(void) const { return is_rc_online_; }
  ///< 右拨杆是否在运行档
  bool isManualControlEnabled(void) const { return is_manual_enabled_; }
  ///< IMU 零漂是否收敛
  bool isImuReady(void) const { return is_imu_ready_; }
  ///< 是否允许电机出力
  bool isOutputAllowed(void) const;
  const GimbalAngle &targetAngle(void) const { return target_; }
  /* --- 诊断用（只读，不参与控制） --- */
  float aimYawNow(void) const { return AimYaw(); }
  float aimPitchNow(void) const { return AimPitch(); }
  bool inputArmed(void) const { return input_armed_; }
  uint32_t workingEntries(void) const { return working_entry_cnt_; }

 private:
  void updateData(void);
  void updateSafety(void);
  void updateStateMachine(void);
  void generateCommand(float dt);
  void zeroOutput(void);
  void enterDead(void);
  void updateCommData(void);
  bool isWorkingConditionMet(void) const;

  /* 控制源判定：本地 DBUS 或对板 C2G 转发，任一有效即可
   * （接收机只接在底盘板时，云台靠 C2G 里的 aim_yaw/aim_pitch 瞄准） */
  bool hasPeerCommand(void) const;
  bool hasControlSource(void) const { return is_rc_online_ || has_peer_command_; }
  float AimYaw(void) const;
  float AimPitch(void) const;

  Config cfg_{};
  Gimbal *gimbal_ = nullptr;
  hello_world::imu::Imu *imu_ = nullptr;
  hello_world::remote_control::DT7 *rc_ = nullptr;
  GimbalChassisComm *comm_ = nullptr;

  LifeState life_state_ = LifeState::kDead;
  uint32_t state_cycles_ = 0;

  bool is_emergency_ = false;
  bool is_rc_online_ = false;      ///< 本板 DBUS 是否在线
  bool has_peer_command_ = false;  ///< 对板 C2G 是否有效且对板在 Working
  bool is_manual_enabled_ = false;
  bool is_imu_ready_ = false;

  bool input_armed_ = false;      ///< 摇杆回中后允许积分目标角
  uint32_t working_entry_cnt_ = 0u;  ///< 进入 Working 的次数（诊断：>1 说明状态在抖）
  uint32_t centered_cycles_ = 0;  ///< 连续回中周期计数

  GimbalAngle target_{};            ///< 摇杆积分出来的关节目标角
  float last_cont_yaw_ = 0.0f;      ///< 上一拍 yaw 连续角（算角速度上报用）
  float yaw_rate_ = 0.0f;           ///< 上报给底盘的 yaw 角速度，单位 rad/s
  bool has_last_cont_yaw_ = false;
};

}  // namespace robot

#endif /* GIMBAL_ROBOT_MODULES_ROBOT_HPP_ */
