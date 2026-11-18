/**
 *******************************************************************************
 * @file      : pid.hpp
 * @brief     : 通用 PID 控制器（位置式）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-18      Chassis        1. 完成编写
 *******************************************************************************
 * @attention :
 *  1. 这是一个可复用的位置式 PID：
 *       u = Kp * e + ∫Ki * e dt + Kd * de/dt
 *     支持积分限幅、输出限幅、积分抗饱和（输出饱和时回退本次积分）。
 *  2. 微分项默认作用于「测量值」而不是「误差」（d_on_measurement = true），
 *     这样目标值阶跃时不会产生微分冲击，模式切换 / 目标突变时更平顺。
 *  3. 所有回路必须显式传入控制周期 dt（单位：s），本类内部不依赖任何硬件。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_PID_PID_HPP_
#define HW_COMPONENTS_ALGORITHMS_PID_PID_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>
#include <cstdint>

namespace hello_world {
namespace pid {

class Pid {
 public:
  Pid(void) = default;

  /**
   * @brief       构造函数
   * @param        kp: 比例系数
   * @param        ki: 积分系数
   * @param        kd: 微分系数
   * @param        i_lim: 积分限幅（正数，积分项被限制在 [-i_lim, i_lim]；<=0 表示不限）
   * @param        out_lim: 输出限幅（正数，输出被限制在 [-out_lim, out_lim]；<=0 表示不限）
   * @retval       None
   * @note        None
   */
  Pid(float kp, float ki, float kd, float i_lim = 0.0f, float out_lim = 0.0f)
      : kp_(kp), ki_(ki), kd_(kd), i_lim_(i_lim), out_lim_(out_lim)
  {
  }

  /**
   * @brief       PID 参数初始化（也可用于在线整定）
   * @param        kp: 比例系数
   * @param        ki: 积分系数
   * @param        kd: 微分系数
   * @param        i_lim: 积分限幅（<=0 表示不限）
   * @param        out_lim: 输出限幅（<=0 表示不限）
   * @retval       None
   * @note        None
   */
  void init(float kp, float ki, float kd, float i_lim = 0.0f, float out_lim = 0.0f)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    i_lim_ = i_lim;
    out_lim_ = out_lim;
    reset();
  }

  /**
   * @brief       复位控制器内部状态（清积分、清微分历史）
   * @retval       None
   * @note        状态机切换 / 失效保护时必须调用，避免残留积分造成电机突变
   */
  void reset(void)
  {
    i_out_ = 0.0f;
    last_err_ = 0.0f;
    last_measure_ = 0.0f;
    out_ = 0.0f;
    is_first_ = true;
  }

  /**
   * @brief       执行一次 PID 计算（微分作用于测量值）
   * @param        err: 误差 = 目标值 - 测量值
   * @param        measure: 测量值（用于微分项）
   * @param        dt: 控制周期，单位：s
   * @retval       PID 输出（已限幅）
   * @note        None
   */
  float calc(float err, float measure, float dt)
  {
    if (!(dt > 0.0f)) {
      dt = kDefaultDt;
    }

    const float p_out = kp_ * err;

    /* 微分：默认作用于测量值，首拍不产生微分冲击 */
    float d_out = 0.0f;
    if (is_first_) {
      last_measure_ = measure;
      last_err_ = err;
      is_first_ = false;
    } else {
      if (d_on_measurement_) {
        d_out = -kd_ * (measure - last_measure_) / dt;
      } else {
        d_out = kd_ * (err - last_err_) / dt;
      }
      last_measure_ = measure;
      last_err_ = err;
    }

    /* 积分（含积分分离与积分限幅）
     * 积分分离：误差过大时暂停积分，避免目标阶跃瞬间积分饱和造成大超调 */
    float delta_i = 0.0f;
    if ((i_sep_thres_ <= 0.0f) || (fabsf(err) <= i_sep_thres_)) {
      delta_i = ki_ * err * dt;
      i_out_ += delta_i;
      if (i_lim_ > 0.0f) {
        if (i_out_ > i_lim_) {
          i_out_ = i_lim_;
        } else if (i_out_ < -i_lim_) {
          i_out_ = -i_lim_;
        }
      }
    }

    float out = p_out + i_out_ + d_out;

    /* 输出限幅 + 积分抗饱和：输出被限幅且积分继续往饱和方向走时回退本次积分 */
    if (out_lim_ > 0.0f) {
      if (out > out_lim_) {
        out = out_lim_;
        if (delta_i > 0.0f) {
          i_out_ -= delta_i;
        }
      } else if (out < -out_lim_) {
        out = -out_lim_;
        if (delta_i < 0.0f) {
          i_out_ -= delta_i;
        }
      }
    }

    out_ = out;
    return out_;
  }

  /**
   * @brief       执行一次 PID 计算（微分作用于误差）
   * @param        err: 误差 = 目标值 - 测量值
   * @param        dt: 控制周期，单位：s
   * @retval       PID 输出（已限幅）
   * @note        None
   */
  float calc(float err, float dt) { return calc(err, err, dt); }

  /* 数据获取 */
  float out(void) const { return out_; }
  float i_out(void) const { return i_out_; }
  float kp(void) const { return kp_; }
  float ki(void) const { return ki_; }
  float kd(void) const { return kd_; }

  /* 参数修改 */
  void set_kp(float kp) { kp_ = kp; }
  void set_ki(float ki) { ki_ = ki; }
  void set_kd(float kd) { kd_ = kd; }
  void set_i_lim(float i_lim) { i_lim_ = i_lim; }
  void set_out_lim(float out_lim) { out_lim_ = out_lim; }
  void set_d_on_measurement(bool on) { d_on_measurement_ = on; }

  /**
   * @brief       设置积分分离阈值
   * @param        thres: 误差绝对值超过该值时暂停积分（<=0 表示不启用）
   * @retval       None
   * @note        目标阶跃瞬间误差很大，此时积分只会造成超调与积分饱和；
   *              误差进入阈值后再启用积分消除稳态误差，可显著减小超调
   */
  void set_i_sep_thres(float thres) { i_sep_thres_ = thres; }

 private:
  static constexpr float kDefaultDt = 0.001f;  ///< 控制周期缺省值，单位：s

  float kp_ = 0.0f;  ///< 比例系数
  float ki_ = 0.0f;  ///< 积分系数
  float kd_ = 0.0f;  ///< 微分系数
  float i_lim_ = 0.0f;    ///< 积分限幅（<=0 不限）
  float out_lim_ = 0.0f;  ///< 输出限幅（<=0 不限）
  float i_sep_thres_ = 0.0f;  ///< 积分分离阈值（<=0 不启用）

  float i_out_ = 0.0f;        ///< 积分项累计值
  float last_err_ = 0.0f;     ///< 上一拍误差
  float last_measure_ = 0.0f; ///< 上一拍测量值
  float out_ = 0.0f;          ///< 上一拍输出
  bool is_first_ = true;      ///< 是否为复位后的第一拍
  bool d_on_measurement_ = true;  ///< 微分是否作用于测量值
};

}  // namespace pid
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_PID_PID_HPP_ */
