/**
 *******************************************************************************
 * @file      :module_state.hpp
 * @brief     : 底盘板 / 云台板共享的状态类型与枚举
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 完成共享类型定义
 *******************************************************************************
 * @attention :
 *  1. 本文件被底盘板、云台板同时编译，只允许「纯类型 / 枚举 / 内联函数」。
 *     严禁 include 任何与具体硬件相关的头文件（motor / HW_can / stm32_hal / base.hpp），
 *     否则另一块板会编译不过。本文件只依赖 <cstdint> 与 <cmath>。
 *  2. 单位约定（全工程统一）：
 *       - 平移速度 vx / vy     ：m/s
 *       - 角速度   w / yaw_rate：rad/s
 *       - 角度     yaw / pitch ：rad
 *       - 轮速                 ：rad/s（电机输出端，已扣除减速比）
 *     坐标约定：x 轴朝前、y 轴朝左、z 轴朝上，右手法则（逆时针为正）；
 *               「前进正方向始终是云台朝向」，因此平移速度 (vx, vy) 一律定义在云台坐标系。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MODULE_STATE_HPP_
#define MODULE_STATE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>
#include <cstdint>

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

namespace robot {

/* 圆周率，避免依赖 base.hpp（base.hpp 会间接引入 HAL） */
static constexpr float kPi = 3.14159265358979f;

/* ============================================================================
 * 1. 生命状态枚举
 *
 *   Dead         上电未就绪 / 急停 / 失控：所有电机指令必须为 0
 *   Resurrection 恢复自检：清残留指令、复位控制器、等待零漂收敛
 *   Working      正常工作：允许按遥控器指令输出
 * ==========================================================================*/
enum class LifeState : uint8_t {
  kDead = 0,          ///< 上电未就绪 / 急停 / 保护
  kResurrection = 1,  ///< 恢复自检中
  kWorking = 2,       ///< 正常工作
};

/* ============================================================================
 * 2. 手动控制源枚举（本次仅遥控器，键鼠为预留扩展）
 * ==========================================================================*/
enum class ControlSource : uint8_t {
  kNone = 0,           ///< 无有效控制源（必须零输出）
  kRemoteControl = 1,  ///< DT7 遥控器
  kKeyboard = 2,       ///< 键鼠（预留，本次不实现）
};

/* ============================================================================
 * 3. 底盘工作模式枚举（行为定义见考核说明 3.1）
 *
 *   kSeparate 分离：底盘与云台解耦，(vx, vy, w) 全部来自遥控器直接映射，
 *                   底盘不自动回正、不自旋
 *   kFollow   跟随：底盘朝向自动跟随云台朝向，w 由「云台朝向 − 底盘朝向」闭环给出
 *   kSpin     小陀螺：底盘绕自身几何中心持续自旋，w 为限幅后的可控常值，
 *                   平移 (vx, vy) 仍以云台朝向为基准
 * ==========================================================================*/
enum class ChassisMode : uint8_t {
  kSeparate = 0,  ///< 分离
  kFollow = 1,    ///< 跟随（随动）
  kSpin = 2,      ///< 小陀螺
};

/* ============================================================================
 * 4. 云台工作模式枚举
 *
 *   kJoint      关节空间：yaw / pitch 直接对关节角做位置闭环（基础项）。
 *               注意：关节空间下，底盘一转云台就跟着转，云台相对底盘的关节角 δ
 *               不会随底盘转动而变化 —— 因此**跟随模式无法收敛**。
 *   kCartesian  笛卡尔空间：云台用自身 IMU 保持绝对朝向（加分项）。
 *               此时底盘转动会让 δ 变化，跟随模式的「车头追云台」才有意义。
 *
 * 底盘板的跟随模式会检查本字段（见 robot::Robot::kFollowRequireAbsoluteGimbal）。
 * ==========================================================================*/
enum class GimbalMode : uint8_t {
  kJoint = 0,      ///< 关节空间
  kCartesian = 1,  ///< 笛卡尔空间（绝对朝向稳定）
};

/* ============================================================================
 * 5. 底盘运动状态：平移速度 (vx, vy) + 旋转角速度 w
 *    单位：m/s、m/s、rad/s
 * ==========================================================================*/
struct ChassisSpeed {
  float vx = 0.0f;  ///< 前进方向平移速度（云台系 x），单位：m/s
  float vy = 0.0f;  ///< 左移方向平移速度（云台系 y），单位：m/s
  float w = 0.0f;   ///< 绕 z 轴角速度（逆时针为正），单位：rad/s

  /* 常用运算 */
  ChassisSpeed operator+(const ChassisSpeed &rhs) const
  {
    return ChassisSpeed{vx + rhs.vx, vy + rhs.vy, w + rhs.w};
  }
  ChassisSpeed operator-(const ChassisSpeed &rhs) const
  {
    return ChassisSpeed{vx - rhs.vx, vy - rhs.vy, w - rhs.w};
  }
  ChassisSpeed operator-() const { return ChassisSpeed{-vx, -vy, -w}; }
  ChassisSpeed operator*(float k) const
  {
    return ChassisSpeed{vx * k, vy * k, w * k};
  }
  ChassisSpeed operator/(float k) const
  {
    return ChassisSpeed{vx / k, vy / k, w / k};
  }
  ChassisSpeed &operator+=(const ChassisSpeed &rhs)
  {
    vx += rhs.vx;
    vy += rhs.vy;
    w += rhs.w;
    return *this;
  }
  ChassisSpeed &operator-=(const ChassisSpeed &rhs)
  {
    vx -= rhs.vx;
    vy -= rhs.vy;
    w -= rhs.w;
    return *this;
  }
  ChassisSpeed &operator*=(float k)
  {
    vx *= k;
    vy *= k;
    w *= k;
    return *this;
  }
  bool isZero(void) const
  {
    return (vx == 0.0f) && (vy == 0.0f) && (w == 0.0f);
  }
  void reset(void)
  {
    vx = 0.0f;
    vy = 0.0f;
    w = 0.0f;
  }
};

/* ============================================================================
 * 6. 云台运动状态：yaw / pitch 两个关节角，单位：rad
 * ==========================================================================*/
struct GimbalAngle {
  float yaw = 0.0f;    ///< yaw 关节角，单位：rad（支持 360° 连续旋转）
  float pitch = 0.0f;  ///< pitch 关节角，单位：rad

  GimbalAngle operator+(const GimbalAngle &rhs) const
  {
    return GimbalAngle{yaw + rhs.yaw, pitch + rhs.pitch};
  }
  GimbalAngle operator-(const GimbalAngle &rhs) const
  {
    return GimbalAngle{yaw - rhs.yaw, pitch - rhs.pitch};
  }
  GimbalAngle operator-() const { return GimbalAngle{-yaw, -pitch}; }
  GimbalAngle operator*(float k) const
  {
    return GimbalAngle{yaw * k, pitch * k};
  }
  GimbalAngle operator/(float k) const
  {
    return GimbalAngle{yaw / k, pitch / k};
  }
  GimbalAngle &operator+=(const GimbalAngle &rhs)
  {
    yaw += rhs.yaw;
    pitch += rhs.pitch;
    return *this;
  }
  GimbalAngle &operator-=(const GimbalAngle &rhs)
  {
    yaw -= rhs.yaw;
    pitch -= rhs.pitch;
    return *this;
  }
  GimbalAngle &operator*=(float k)
  {
    yaw *= k;
    pitch *= k;
    return *this;
  }
};

/* ============================================================================
 * 7. 角度归一化小工具（不依赖 base.hpp）
 * ==========================================================================*/

/**
 * @brief       把角度归一化到 [-π, π)
 * @param        angle: 待归一化角度，单位：rad
 * @retval       归一化后的角度，单位：rad
 * @note        用于「359° → 1° 必须当成 +2°」的连续旋转处理（见考核说明 3.5）
 */
inline float NormalizeAngleRad(float angle)
{
  float res = std::fmod(angle + kPi, 2.0f * kPi);
  if (res < 0.0f) {
    res += 2.0f * kPi;
  }
  return res - kPi;
}

/**
 * @brief       把输入量限制到 [min_lim, max_lim]
 * @param        x: 待限幅量
 * @retval       限幅后的值
 * @note        不依赖 base.hpp 的 Bound，避免共享文件引入硬件依赖
 */
inline float ClampValue(float x, float min_lim, float max_lim)
{
  if (x > max_lim) {
    return max_lim;
  }
  if (x < min_lim) {
    return min_lim;
  }
  return x;
}

}  // namespace robot

#endif /* MODULE_STATE_HPP_ */
