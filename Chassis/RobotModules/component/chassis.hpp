/**
 *******************************************************************************
 * @file      :chassis.hpp
 * @brief     : 底盘运动模块（工作模式 + 麦轮逆解 + 轮速闭环）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 完成编写
 *******************************************************************************
 * @attention :
 *  1. 本类只负责「底盘」这一件事：把操作手输入按工作模式转成底盘速度指令，
 *     先做云台系 → 车体系的旋转（考核说明 3.2），再做麦克纳姆轮逆解（45° 滚子角），
 *     最后对 4 个轮做速度环闭环，输出电机报文原始值。
 *  2. 所有输出都必须经过 Robot 状态机：本类自己不做安全判断，只负责「被允许输出时
 *     按指令输出」，失效时由 Robot 调用 reset() 立刻归零。
 *  3. 单位：vx/vy 为 m/s，w 为 rad/s，轮速为 rad/s（电机输出端）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_ROBOT_MODULES_CHASSIS_HPP_
#define CHASSIS_ROBOT_MODULES_CHASSIS_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "chassis_ik_solver.hpp"
#include "module_state.hpp"
#include "motor_base.hpp"
#include "pid.hpp"

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

namespace robot {

/* 操作手输入（已经过遥控器摇杆 → 物理量换算，单位见注释） */
struct ChassisInput {
  float vx = 0.0f;         ///< 云台系前进速度，单位：m/s
  float vy = 0.0f;         ///< 云台系左移速度，单位：m/s
  float w_manual = 0.0f;   ///< 分离模式下底盘角速度（拨轮直接映射），单位：rad/s
  float w_spin = 0.0f;     ///< 小陀螺模式的自旋调制量（拨轮），单位：rad/s（叠加在基准转速上）
  /** 云台是否处于「绝对朝向」模式。跟随环靠它保证收敛：
   *  关节空间下底盘一转云台就跟着转，δ 不变，跟随环会变成匀速自转（失控）。 */
  bool follow_rotation_allowed = true;
  /* 底盘自身角速度（IMU，rad/s，逆时针为正），跟随环阻尼项用。
   * 恒为 0 时跟随退化为纯 P —— 会形成极限环（顺逆时针来回震荡），仅调试用。 */
  float yaw_rate = 0.0f;

  void reset(void)
  {
    vx = 0.0f;
    vy = 0.0f;
    w_manual = 0.0f;
    w_spin = 0.0f;
    follow_rotation_allowed = true;
    yaw_rate = 0.0f;
  }
};

/* 底盘运动模块 */
class Chassis {
 public:
  /* 轮子数量，与 chassis_ik_solver::kWheelNum 一致 */
  static constexpr int kWheelNum = 4;

  /* ---------------- 速度 / 加速度限幅（安全红线：限速，不允许多快就多快） ----------------
   * 当前是**调试期偏保守**的取值，熟练后可上调（例如 kMaxVelX/kMaxVelY 2.5）。
   * 轮子架空时任何速度都显得很快，先用小值把方向、闭环都验通再放速度。 */
  static constexpr float kMaxVelX = 1.2f;        ///< 前进 / 后退最大速度，单位：m/s
  static constexpr float kMaxVelY = 1.2f;        ///< 左移 / 右移最大速度，单位：m/s
  static constexpr float kMaxOmegaManual = 1.2f; ///< 分离模式最大角速度，单位：rad/s
  static constexpr float kMaxSpinOmega = 2.5f;   ///< 小陀螺最大自旋角速度，单位：rad/s
  static constexpr float kMaxWheelVel = 20.0f;   ///< 单轮最大轮速，单位：rad/s（约 190rpm ≈ 1.56 m/s）
  static constexpr float kMaxAccel = 6.0f;       ///< 平移加速度限幅，单位：m/s²
  static constexpr float kMaxAngAccel = 20.0f;   ///< 角加速度限幅，单位：rad/s²

  /* ---------------- 小陀螺（持续自转 + 拨轮调制） ---------------- */
  static constexpr float kSpinBaseRate = 1.2f;   ///< 基准自转速度，单位：rad/s
  static constexpr float kSpinWheelGain = 1.2f;  ///< 拨轮对自转速度的调制幅度，单位：rad/s

  /* ---------------- 跟随模式闭环（δ → w） ---------------- */
  static constexpr float kFollowKp = 4.5f;        ///< 跟随比例系数，单位：1/s
  static constexpr float kFollowKi = 0.0f;        ///< 跟随积分系数
  /* 跟随阻尼项：w = kp*δ − kd*ω_chassis。ω 取底盘自身 IMU 角速度（低通滤波后）。
 * kd 取 0.4 量级；只给 P 会因为轮速环 ~0.1s 滞后 + G2C 10ms 传输延迟形成极限环，
 * 表现为车在顺/逆时针之间来回震荡（实测确认）。 */
static constexpr float kFollowKd = 0.52f;       ///< 跟随阻尼系数，单位：rad/s ÷ (rad/s)
  static constexpr float kFollowOutLim = 4.2f;    ///< 跟随角速度输出限幅，单位：rad/s
  static constexpr float kFollowDeadBand = 0.012f;///< 跟随死区，单位：rad（约 0.7°）

  /* ---------------- 电流安全限幅 ---------------- */
  ///< 四轮电流绝对值合计上限，单位：A（M3508 单轮连续 20A，电池/分电板撑不住四轮同时拉满）
  static constexpr float kWheelCurrentSumLimit = 25.0f;

  /* 控制周期缺省值，单位：s（与 TIM6 的 1ms 一致） */
  static constexpr float kDefaultDt = 0.001f;

  Chassis(void);

  /* ---------------- 模块注册（由 MainTaskInit 完成） ---------------- */

  /**
   * @brief       注册第 idx 号轮电机
   * @param        idx: 轮序号（chassis_ik_solver::WheelIndex）
   * @param        motor: 电机对象指针
   * @retval       None
   * @note        None
   */
  void registerMotor(int idx, hello_world::motor::Motor *motor);

  /**
   * @brief       注册第 idx 号轮的速度环 PID
   * @param        idx: 轮序号
   * @param        pid: PID 对象指针（参数已在 Instance 层整定）
   * @retval       None
   * @note        None
   */
  void registerPid(int idx, hello_world::pid::Pid *pid);

  /**
   * @brief       注册运动学逆解器
   * @param        solver: 逆解器对象指针
   * @retval       None
   * @note        None
   */
  void registerIkSolver(hello_world::chassis_ik_solver::ChassisIkSolver *solver);

  ///< 电机 / PID / 逆解器是否都已注册完毕
  bool isReady(void) const;

  /* ---------------- 周期接口 ---------------- */

  /**
   * @brief       设置本周期操作手输入
   * @param        input: 操作手输入（单位见 ChassisInput 注释）
   * @retval       None
   * @note        None
   */
  void setInput(const ChassisInput &input) { input_ = input; }

  /**
   * @brief       周期性更新（1ms）：模式解算 → 旋转变换 → 逆解 → 轮速闭环 → 下发
   * @param        mode: 当前底盘工作模式
   * @param        delta: 云台相对底盘的偏航角 δ（云台朝向 − 车头朝向），单位：rad
   * @param        dt: 控制周期，单位：s
   * @retval       None
   * @note        调用前必须保证状态机允许输出；不允许输出时应调用 reset()
   */
  void update(ChassisMode mode, float delta, float dt);

  /**
   * @brief       失效归零：清空目标速度、复位全部 PID、4 个电机立刻 setInput(0)
   * @retval       None
   * @note        急停 / 离线 / 未进入 Working 时必须调用（硬性验收项）
   */
  void reset(void);

  /**
   * @brief       四轮电流合计限幅（等比例缩放，保持配比与方向）
   * @retval       None
   * @note        原地修改 wheel_output_
   */
  void limitCurrentSum(void);

  /* ---------------- 数据获取（调试用） ---------------- */

  ///< 本周期最终下发的底盘速度（车体系，已含模式解算与限幅）
  const ChassisSpeed &chassisSpeed(void) const { return chassis_speed_; }
  ///< 4 个轮的目标轮速，单位：rad/s
  const float *wheelTargetVel(void) const { return wheel_target_vel_; }
  ///< 4 个轮的实际轮速，单位：rad/s
  const float *wheelActualVel(void) const { return wheel_actual_vel_; }
  ///< 4 个轮的控制量（电机报文原始值）
  const float *wheelOutput(void) const { return wheel_output_; }
  ///< 上一次使用的 δ，单位：rad
  float delta(void) const { return delta_; }

 private:
  /**
   * @brief       按工作模式把操作手输入解算成底盘速度指令（云台系 + 角速度）
   * @param        mode: 当前底盘工作模式
   * @param        raw: 输出，未经限幅的底盘速度指令
   * @retval       None
   * @note        None
   */
  void solveMode(ChassisMode mode, ChassisSpeed *raw);

  /**
   * @brief       对速度指令做加速度限幅（保证模式切换 / 摇杆突变时平顺）
   * @param        target: 目标速度指令
   * @param        dt: 控制周期，单位：s
   * @retval       限幅后的速度指令
   * @note        None
   */
  ChassisSpeed slewLimit(const ChassisSpeed &target, float dt);

  /**
   * @brief       云台系 → 车体系的平移速度旋转（考核说明 3.2）
   * @param        vx_gimbal: 云台系前进速度，单位：m/s
   * @param        vy_gimbal: 云台系左移速度，单位：m/s
   * @param        delta: 云台相对底盘的偏航角 δ，单位：rad
   * @param        vx_chassis: 输出车体系前进速度，单位：m/s
   * @param        vy_chassis: 输出车体系左移速度，单位：m/s
   * @retval       None
   * @note        None
   */
  static void rotateGimbalToChassis(float vx_gimbal, float vy_gimbal, float delta,
                                    float *vx_chassis, float *vy_chassis);

  /* 注册的模块 */
  hello_world::motor::Motor *motors_[kWheelNum] = {nullptr, nullptr, nullptr, nullptr};
  hello_world::pid::Pid *pids_[kWheelNum] = {nullptr, nullptr, nullptr, nullptr};
  hello_world::chassis_ik_solver::ChassisIkSolver *ik_solver_ = nullptr;

  /* 跟随模式角速度环（δ → w） */
  hello_world::pid::Pid follow_pid_;

  /* 状态 */
  ChassisInput input_;              ///< 本周期的操作手输入
  ChassisSpeed cmd_smoothed_;       ///< 加速度限幅后的速度指令（云台系平移 + 车体系角速度）
  ChassisSpeed chassis_speed_;      ///< 本周期最终下发的车体系速度
  float wheel_target_vel_[kWheelNum] = {0.0f, 0.0f, 0.0f, 0.0f};
  float wheel_actual_vel_[kWheelNum] = {0.0f, 0.0f, 0.0f, 0.0f};
  float wheel_output_[kWheelNum] = {0.0f, 0.0f, 0.0f, 0.0f};
  float delta_ = 0.0f;              ///< 上一次使用的 δ，单位：rad
  float dt_ = kDefaultDt;           ///< 本周期控制周期，单位：s
};

}  // namespace robot

#endif /* CHASSIS_ROBOT_MODULES_CHASSIS_HPP_ */
