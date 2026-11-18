/**
 *******************************************************************************
 * @file      : chassis_ik_solver.hpp
 * @brief     : 麦克纳姆轮底盘运动学逆解
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-18      Chassis        1. 完成编写
 *  V1.1.0      2026-11-18      Chassis        2. 轮位顺序 / CAN id / 方向按实车标定表修正
 *******************************************************************************
 * @attention :
 *  1. 结构：4 个 M3508 驱动 4 个麦克纳姆轮（滚子 45°，X 型布置），轮心对称布置。
 *  2. 坐标约定：x 轴朝前、y 轴朝左、z 轴朝上，逆时针为正（右手法则）。
 *  3. 【轮位编号 —— 必须与 ins_motor.cpp 的 CAN id 表一致】
 *       索引  轮位        CAN id    方向
 *       0     FL 左前      1        反向
 *       1     FR 右前      4        正向
 *       2     RL 左后      2        反向
 *       3     RR 右后      3        正向
 *     这张表来自实车标定（逐个手转轮子看 0x201~0x204 哪一帧反馈变化），
 *     **不是**按 id 顺序排的，改动必须两边同时改。
 *  4. 逆解推导：接触点速度 v_p = (vx - w*y_i, vy + w*x_i)，麦轮约束要求
 *     「v_p 在车轮驱动方向上的分量 = R*ω」，其中驱动方向
 *       左前/右后（"左旋"轮）: n = ( 1, -1)/√2
 *       右前/左后（"右旋"轮）: n = ( 1,  1)/√2
 *     代入后 √2 约掉，得到（k = 半轴距 + 半轮距 = a + b）：
 *       FL = vx - vy - k*w      FR = vx + vy + k*w
 *       RL = vx + vy - k*w      RR = vx - vy + k*w
 *     符号结构自检：纯 vx → 四轮同向同幅；纯 vy → 对角同向（FL==RR、FR==RL）；
 *                   纯 w  → 同侧同向（FL==RL、FR==RR），左右反向。
 *  5. 轮速单位：rad/s（电机输出端，已扣除减速比 3591/187）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_CHASSIS_IK_SOLVER_HPP_
#define HW_COMPONENTS_ALGORITHMS_CHASSIS_IK_SOLVER_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

namespace hello_world {
namespace chassis_ik_solver {

/* 轮序号：必须与 ins_motor.cpp 的 CAN id / 方向表、chassis.cpp 的 PID 顺序一致。
 * 顺序为 FL / FR / RL / RR（实车标定表的自然顺序），对应的 CAN id 是 1 / 4 / 2 / 3。 */
enum WheelIndex : uint8_t {
  kWheelFl = 0,  ///< 左前（CAN id 1，反向）
  kWheelFr = 1,  ///< 右前（CAN id 4，正向）
  kWheelRl = 2,  ///< 左后（CAN id 2，反向）
  kWheelRr = 3,  ///< 右后（CAN id 3，正向）
  kWheelNum = 4, ///< 轮子数量
};

class ChassisIkSolver {
 public:
  /* 实车几何参数，取自考核说明 6.2（以实机测量为准） */
  static constexpr float kDefaultWheelRadius = 0.07786f;      ///< 轮半径 R，单位：m
  static constexpr float kDefaultHalfWheelbase = 0.15338f;    ///< 半轴距 L/2，单位：m
  static constexpr float kDefaultHalfTrack = 0.15338f;        ///< 半轮距 W/2，单位：m

  ChassisIkSolver(void) = default;

  /**
   * @brief       构造函数
   * @param        wheel_radius: 轮半径 R，单位：m
   * @param        half_wheelbase: 半轴距 L/2（前后轮心到几何中心的纵向距离），单位：m
   * @param        half_track: 半轮距 W/2（左右轮心到几何中心的横向距离），单位：m
   * @retval       None
   * @note        None
   */
  ChassisIkSolver(float wheel_radius, float half_wheelbase, float half_track)
      : wheel_radius_(wheel_radius),
        half_wheelbase_(half_wheelbase),
        half_track_(half_track)
  {
  }

  /**
   * @brief       底盘速度 → 4 个轮的轮速
   * @param        vx: 车体系前进速度，单位：m/s
   * @param        vy: 车体系左移速度，单位：m/s
   * @param        w: 绕 z 轴角速度（逆时针为正），单位：rad/s
   * @param        wheel_vel: 输出数组，长度 kWheelNum，单位：rad/s
   * @retval       None
   * @note        输入必须是「车体系」速度；云台系 → 车体系的旋转由 Chassis 模块在此之前完成
   */
  void solve(float vx, float vy, float w, float wheel_vel[kWheelNum]) const
  {
    /* k = a + b：麦克纳姆轮自旋项的等效力臂（不是半对角线长度） */
    const float k = half_wheelbase_ + half_track_;
    const float inv_r = 1.0f / wheel_radius_;

    wheel_vel[kWheelFl] = (vx - vy - k * w) * inv_r;
    wheel_vel[kWheelFr] = (vx + vy + k * w) * inv_r;
    wheel_vel[kWheelRl] = (vx + vy - k * w) * inv_r;
    wheel_vel[kWheelRr] = (vx - vy + k * w) * inv_r;
  }

  /**
   * @brief       4 个轮的轮速 → 底盘速度（正解，用于自检 / 调试）
   * @param        wheel_vel: 输入数组，长度 kWheelNum，单位：rad/s
   * @param        vx: 输出车体系前进速度，单位：m/s
   * @param        vy: 输出车体系左移速度，单位：m/s
   * @param        w: 输出绕 z 轴角速度，单位：rad/s
   * @retval       None
   * @note        None
   */
  void solveForward(const float wheel_vel[kWheelNum], float *vx, float *vy,
                    float *w) const
  {
    const float k = half_wheelbase_ + half_track_;
    const float r = wheel_radius_;

    const float v_fl = wheel_vel[kWheelFl];
    const float v_fr = wheel_vel[kWheelFr];
    const float v_rl = wheel_vel[kWheelRl];
    const float v_rr = wheel_vel[kWheelRr];

    if (vx != nullptr) {
      *vx = (v_fl + v_fr + v_rl + v_rr) * 0.25f * r;
    }
    if (vy != nullptr) {
      *vy = (-v_fl + v_fr + v_rl - v_rr) * 0.25f * r;
    }
    if (w != nullptr) {
      *w = (-v_fl + v_fr - v_rl + v_rr) * 0.25f * r / k;
    }
  }

  /* 几何参数访问 */
  float wheel_radius(void) const { return wheel_radius_; }
  float half_wheelbase(void) const { return half_wheelbase_; }
  float half_track(void) const { return half_track_; }

 private:
  float wheel_radius_ = kDefaultWheelRadius;        ///< 轮半径 R，单位：m
  float half_wheelbase_ = kDefaultHalfWheelbase;    ///< 半轴距 L/2，单位：m
  float half_track_ = kDefaultHalfTrack;            ///< 半轮距 W/2，单位：m
};

}  // namespace chassis_ik_solver
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_CHASSIS_IK_SOLVER_HPP_ */
