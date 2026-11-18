/**
 * @file  imu.hpp（等价性测试用 mock）
 * @brief 只提供 gimbal.cpp 用到的两个接口：isReady() / yaw()
 */
#ifndef EQUIV_MOCK_IMU_HPP_
#define EQUIV_MOCK_IMU_HPP_

namespace hello_world {
namespace imu {

class Imu {
 public:
  void setReady(bool ready) { ready_ = ready; }
  bool isReady(void) const { return ready_; }

  void setYaw(float yaw) { yaw_ = yaw; }
  float yaw(void) const { return yaw_; }

 private:
  bool ready_ = false;
  float yaw_ = 0.0f;
};

}  // namespace imu
}  // namespace hello_world

#endif /* EQUIV_MOCK_IMU_HPP_ */
