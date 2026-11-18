/**
 *******************************************************************************
 * @file      : imu.hpp
 * @brief     : IMU 设备封装（BMI088 + Mahony + 开机零漂）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-12-01      Jinletian       1. 初版编写完成
 *  V2.0.0      2026-09-18      Hello World     1. 考核骨架：掏空 Imu 封装类
 *  V3.0.0      2026-11-18      Chassis         1. 完成零漂 + 姿态角封装
 *******************************************************************************
 * @attention :
 *  1. init() 内部会阻塞（BMI088 软复位、寄存器配置），只允许在 MainTaskInit() 中调用，
 *     严禁在中断 / 1ms 周期里调用。
 *  2. update() 每个控制周期（1ms）调用一次：采样 → 扣零漂 → Mahony 更新。
 *  3. 零漂未收敛（isReady() == false）前，状态机不得进入 Working（考核说明 3.4）。
 *  4. 坐标约定：x 前、y 左、z 上，右手法则；姿态角为相对开机姿态的欧拉角（Z-Y-X），
 *     单位 rad。
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_IMU_IMU_HPP_
#define HW_COMPONENTS_DEVICES_IMU_IMU_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "BMI088.hpp"
#include "mahony.hpp"

/* ============================================================================
 * Imu 封装类：组合 BMI088（芯片原始读取）与 Mahony（姿态解算）。
 *   - 开机静止采样求陀螺零漂（角速度均值），之后每帧扣除该零漂再送入 Mahony；
 *   - 对外输出 roll() / pitch() / yaw()（rad，相对开机姿态）；
 *   - isReady() 用于状态机判断「零漂是否收敛」。
 * ==========================================================================*/

namespace hello_world {
namespace imu {

class Imu {
 public:
  /* 零漂标定参数 */
  static constexpr uint32_t kCalibSampleNum = 2000u;  ///< 需要的静止采样点数（1ms 周期 → 2s）
  static constexpr float kCalibQuietThres = 0.30f;    ///< 静止判定阈值，单位：rad/s

  /* Mahony 姿态解算参数（零漂已单独扣除，Ki 取 0 即可） */
  static constexpr float kSampleFreq = 1000.0f;  ///< 采样频率，单位：Hz（与 1ms 控制周期一致）
  static constexpr float kMahonyKp = 1.00f;      ///< 比例系数
  static constexpr float kMahonyKi = 0.00f;      ///< 积分系数

  Imu(void) = default;
  ~Imu(void) = default;

  /**
   * @brief       IMU 初始化（阻塞，只能在上电初始化阶段调用一次）
   * @retval      是否初始化成功（芯片自检通过）
   * @note        会自动尝试两种片选引脚组合，兼容硬件上加速度计/陀螺仪片选接反的情况
   */
  bool init(void);

  /**
   * @brief       周期性更新（1ms 一次）：采样 → 零漂估计 → Mahony 解算
   * @retval      None
   * @note        未初始化成功时直接返回，不产生任何姿态输出
   */
  void update(void);

  /* ---------------- 状态查询 ---------------- */

  ///< 零漂是否已收敛（收敛前状态机不得进入 Working）
  bool isReady(void) const { return is_ready_; }
  ///< 芯片是否初始化成功
  bool isChipOk(void) const { return chip_ok_; }
  ///< 加速度计 / 陀螺仪片选是否与默认配置相反（便于排查接线）
  bool isCsSwapped(void) const { return cs_swapped_; }
  ///< 零漂标定已完成采样点数，便于调试观察收敛进度
  uint32_t calibSampleNum(void) const { return calib_cnt_; }

  /* ---------------- 姿态角（rad，相对开机姿态） ---------------- */

  float roll(void) const { return euler_[0]; }
  float pitch(void) const { return euler_[1]; }
  float yaw(void) const { return euler_[2]; }
  const float *euler(void) const { return euler_; }

  /* ---------------- 原始数据（已扣除零漂） ---------------- */

  const float *acc(void) const { return acc_; }
  const float *gyro(void) const { return gyro_; }
  float gyroBias(uint8_t axis) const
  {
    return (axis < 3u) ? gyro_bias_[axis] : 0.0f;
  }

 private:
  /**
   * @brief       用指定片选组合初始化 BMI088 并做芯片自检
   * @param        acc_cs_port / acc_cs_pin: 加速度计片选
   * @param        gyro_cs_port / gyro_cs_pin: 陀螺仪片选
   * @retval       BMI088 错误状态位（0 表示全部通过）
   * @note        None
   */
  uint8_t initChipWith(GPIO_TypeDef *acc_cs_port, uint32_t acc_cs_pin,
                       GPIO_TypeDef *gyro_cs_port, uint32_t gyro_cs_pin);

  /**
   * @brief       重置零漂标定状态
   * @retval      None
   * @note        None
   */
  void resetCalibration(void);

  BMI088 chip_;          ///< BMI088 芯片驱动
  ahrs::Mahony ahrs_;    ///< Mahony 姿态解算

  bool is_init_ = false;    ///< init() 是否已执行
  bool chip_ok_ = false;    ///< 芯片是否自检通过
  bool cs_swapped_ = false; ///< 片选是否使用了交换后的组合
  bool is_ready_ = false;   ///< 零漂是否收敛

  float acc_[3] = {0.0f, 0.0f, 0.0f};   ///< 加速度，单位：m/s²
  float gyro_[3] = {0.0f, 0.0f, 0.0f};  ///< 角速度（已扣零漂），单位：rad/s
  float euler_[3] = {0.0f, 0.0f, 0.0f}; ///< [roll pitch yaw]，单位：rad

  float gyro_bias_[3] = {0.0f, 0.0f, 0.0f};  ///< 陀螺零漂估计，单位：rad/s
  float bias_sum_[3] = {0.0f, 0.0f, 0.0f};   ///< 零漂累加和，单位：rad/s
  uint32_t calib_cnt_ = 0u;                  ///< 已累计的静止采样点数
};

}  // namespace imu
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_IMU_IMU_HPP_ */
