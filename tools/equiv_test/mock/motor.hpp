/**
 * @file  motor.hpp（等价性测试用 mock）
 * @brief 只提供 gimbal.cpp 用到的两个接口：angle() / setInput()
 * @note  真实 motor.hpp 依赖 HAL，host 上编译不了，所以用同名头遮蔽。
 */
#ifndef EQUIV_MOCK_MOTOR_HPP_
#define EQUIV_MOCK_MOTOR_HPP_

namespace hello_world {
namespace motor {

class Motor {
 public:
  virtual ~Motor() = default;

  ///< 由测试脚本设定"当前真实角度"
  void setAngle(float angle) { angle_ = angle; }
  ///< 云台模块读到的反馈角度
  float angle(void) const { return angle_; }

  ///< 云台模块下发的力矩（记录最近一次，测试据此比对）
  virtual void setInput(float input) { last_input_ = input; }
  float lastInput(void) const { return last_input_; }

 private:
  float angle_ = 0.0f;
  float last_input_ = 0.0f;
};

}  // namespace motor
}  // namespace hello_world

#endif /* EQUIV_MOCK_MOTOR_HPP_ */
