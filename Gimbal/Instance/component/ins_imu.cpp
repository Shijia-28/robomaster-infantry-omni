/**
 *******************************************************************************
 * @file      :ins_imu.cpp
 * @brief     : 云台板 IMU 实例化
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-11-18      Chassis        1. 完成实例化
 *******************************************************************************
 * @attention :
 *  这里只负责静态定义并返回地址，芯片初始化（阻塞）与零漂标定分别在
 *  MainTaskInit()（调 Imu::init）与 1ms 周期（调 Imu::update）中完成。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_imu.hpp"

#include "imu.hpp"

namespace {

/* 函数内静态：首次 GetImu() 时构造（避免 main() 之前跑构造） */
hello_world::imu::Imu *ImuInstance(void)
{
  static hello_world::imu::Imu imu;
  return &imu;
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/
hello_world::imu::Imu *GetImu()
{
  return ImuInstance();
}
