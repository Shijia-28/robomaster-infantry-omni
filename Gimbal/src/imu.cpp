/**
 *******************************************************************************
 * @file      : imu.cpp
 * @brief     : IMU 设备封装实现（BMI088 + Mahony + 开机零漂）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-18      Chassis         1. 完成编写
 *******************************************************************************
 * @attention :
 *  1. 硬件接线（CubeMX 已固定，见 Core/Src/spi.c、gpio.c）：
 *       SPI1：PB3=SCK、PB4=MISO、PA7=MOSI，SPI_MODE_MASTER、8bit、CPOL=Low、CPHA=1Edge
 *       片选：PA4 与 PB0（CubeMX 里都是推挽输出、上电默认拉高）
 *     若硬件上两个片选接反，init() 会自动交换再试一次，并可通过 isCsSwapped() 观察。
 *  2. 安装旋转矩阵默认取单位阵（芯片 x/y/z 与车体前/左/上一致）。若实车 BMI088 是
 *     旋转安装的，请修改 kRotMatFlatten 后重新标定（只影响姿态角，不影响零漂标定）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "imu.hpp"

#include <cmath>

#include "gpio.h"
#include "spi.h"

namespace hello_world {
namespace imu {

namespace {

/* 片选引脚（来自 CubeMX gpio.c：PA4 / PB0 均为推挽输出、上电默认高电平） */
GPIO_TypeDef *const kAccCsPortDefault = GPIOA;
constexpr uint32_t kAccCsPinDefault = GPIO_PIN_4;
GPIO_TypeDef *const kGyroCsPortDefault = GPIOB;
constexpr uint32_t kGyroCsPinDefault = GPIO_PIN_0;

/* BMI088 安装旋转矩阵（展平，行优先）：单位阵 = 芯片坐标系与车体坐标系重合 */
constexpr float kRotMatFlatten[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f,
};

/* BMI088 设备配置：加速度 ±6g / 1600Hz，陀螺 ±1000dps / 1000Hz */
static const BMI088Config kChipConfig = [] {
  BMI088Config cfg;
  cfg.acc_range = kBMI088AccRange6G;
  cfg.acc_odr = kBMI088AccOdr1600;
  cfg.acc_osr = kBMI088AccOsr4;
  cfg.gyro_range = kBMI088GyroRange1000Dps;
  cfg.gyro_odr_fbw = kBMI088GyroOdrFbw1000_116;
  return cfg;
}();

}  // namespace

bool Imu::init(void)
{
  /* 只允许执行一次 */
  if (is_init_) {
    return chip_ok_;
  }
  is_init_ = true;

  /* Mahony 初始化：用单位四元数作为起始姿态（相对开机姿态） */
  ahrs_.init(kSampleFreq, kMahonyKp, kMahonyKi);

  uint8_t err = initChipWith(kAccCsPortDefault, kAccCsPinDefault,
                             kGyroCsPortDefault, kGyroCsPinDefault);
  cs_swapped_ = false;

  /* 加速度计 / 陀螺仪任一「找不到」，说明片选接反，交换后重试一次 */
  const uint8_t not_found_mask =
      static_cast<uint8_t>(kBMI088ErrStateAccNotFound) |
      static_cast<uint8_t>(kBMI088ErrStateGyroNotFound);
  if ((err & not_found_mask) != 0u) {
    const uint8_t err_swapped = initChipWith(kGyroCsPortDefault, kGyroCsPinDefault,
                                             kAccCsPortDefault, kAccCsPinDefault);
    if (err_swapped == static_cast<uint8_t>(kBMI088ErrStateNoErr)) {
      err = err_swapped;
      cs_swapped_ = true;
    }
  }

  chip_ok_ = (err == static_cast<uint8_t>(kBMI088ErrStateNoErr));
  resetCalibration();
  return chip_ok_;
}

void Imu::update(void)
{
  if (!chip_ok_) {
    return;
  }

  /* 1. 读取原始数据（加速度含重力项，单位 m/s²；角速度单位 rad/s） */
  chip_.getData(acc_, gyro_, nullptr);

  /* 2. 零漂标定：静止采样求角速度均值 */
  if (!is_ready_) {
    const float gyro_norm =
        sqrtf(gyro_[0] * gyro_[0] + gyro_[1] * gyro_[1] + gyro_[2] * gyro_[2]);
    if (gyro_norm > kCalibQuietThres) {
      /* 车体不静止（正在被搬动 / 上电时在动）：清空重来，绝不带着错误零漂进入 Working */
      resetCalibration();
    } else {
      bias_sum_[0] += gyro_[0];
      bias_sum_[1] += gyro_[1];
      bias_sum_[2] += gyro_[2];
      ++calib_cnt_;
      const float inv_cnt = 1.0f / static_cast<float>(calib_cnt_);
      gyro_bias_[0] = bias_sum_[0] * inv_cnt;
      gyro_bias_[1] = bias_sum_[1] * inv_cnt;
      gyro_bias_[2] = bias_sum_[2] * inv_cnt;
      if (calib_cnt_ >= kCalibSampleNum) {
        is_ready_ = true;
      }
    }
  }

  /* 3. 扣除零漂 */
  float gyro_no_bias[3];
  gyro_no_bias[0] = gyro_[0] - gyro_bias_[0];
  gyro_no_bias[1] = gyro_[1] - gyro_bias_[1];
  gyro_no_bias[2] = gyro_[2] - gyro_bias_[2];
  gyro_[0] = gyro_no_bias[0];
  gyro_[1] = gyro_no_bias[1];
  gyro_[2] = gyro_no_bias[2];

  /* 4. Mahony 姿态更新（加速度含重力项；全 0 时算法内部会跳过加速度修正） */
  ahrs_.update(acc_, gyro_no_bias);
  ahrs_.getEulerAngle(euler_);
}

/* Private function definitions ----------------------------------------------*/
uint8_t Imu::initChipWith(GPIO_TypeDef *acc_cs_port, uint32_t acc_cs_pin,
                          GPIO_TypeDef *gyro_cs_port, uint32_t gyro_cs_pin)
{
  BMI088HWConfig hw_config;
  hw_config.hspi = &hspi1;
  hw_config.acc_cs_port = acc_cs_port;
  hw_config.acc_cs_pin = acc_cs_pin;
  hw_config.gyro_cs_port = gyro_cs_port;
  hw_config.gyro_cs_pin = gyro_cs_pin;

  chip_.init(hw_config, kRotMatFlatten, kChipConfig);

  /* 自检测会额外消耗时间，这里关闭；只做「找得到芯片 + 配置回读正确」的自检 */
  return static_cast<uint8_t>(chip_.imuInit(false));
}

void Imu::resetCalibration(void)
{
  bias_sum_[0] = 0.0f;
  bias_sum_[1] = 0.0f;
  bias_sum_[2] = 0.0f;
  gyro_bias_[0] = 0.0f;
  gyro_bias_[1] = 0.0f;
  gyro_bias_[2] = 0.0f;
  calib_cnt_ = 0u;
  is_ready_ = false;
}

}  // namespace imu
}  // namespace hello_world
