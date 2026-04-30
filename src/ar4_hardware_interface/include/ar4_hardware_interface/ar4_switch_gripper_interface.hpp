#pragma once

#include <math.h>

#include <boost/scoped_ptr.hpp>
#include <chrono>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "ar4_hardware_interface/arduino_nano_driver.hpp"

using namespace hardware_interface;

namespace ar4_hardware_interface {
class ARSwitchGripperHWInterface : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ARSwitchGripperHWInterface);

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  rclcpp::Logger logger_ = rclcpp::get_logger("ar_switch_gripper_hw_interface");
  rclcpp::Clock clock_ = rclcpp::Clock(RCL_ROS_TIME);
  double servo_arm_length_ = 0.023;  // meters
  // offset in degrees for the zero position, in case the servo can't reach
  // 0 degrees due to mechanical tolerance issues.
  double zero_deg_offset_ = 2;

  ArduinoNanoDriver driver_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  double position_command_;

};
}  // namespace ar4_hardware_interface

// #pragma once

// #include <hardware_interface/system_interface.hpp>
// #include <rclcpp/rclcpp.hpp>

// namespace ar4_hardware_interface {

// class ARSwitchGripperHWInterface : public hardware_interface::SystemInterface {
//  public:
//   RCLCPP_SHARED_PTR_DEFINITIONS(ARSwitchGripperHWInterface);

//   hardware_interface::CallbackReturn on_init(
//       const hardware_interface::HardwareInfo& info) override;
//   std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
//   std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
//   hardware_interface::CallbackReturn on_activate(
//       const rclcpp_lifecycle::State& previous_state) override;
//   hardware_interface::CallbackReturn on_deactivate(
//       const rclcpp_lifecycle::State& previous_state) override;
//   hardware_interface::return_type read(
//       const rclcpp::Time& time, const rclcpp::Duration& period) override;
//   hardware_interface::return_type write(
//       const rclcpp::Time& time, const rclcpp::Duration& period) override;

//  private:
//   rclcpp::Logger logger_ = rclcpp::get_logger("ar_switch_gripper_hw_interface");
//   rclcpp::Clock clock_ = rclcpp::Clock(RCL_ROS_TIME);

//   // Joint state (position reported back to ros2_control)
//   double position_ = 0.0;   // 回報給 controller 的位置
//   double velocity_ = 0.0;

//   // Command from controller
//    position_command_ = 0.0;

//   // Threshold: below this → OPEN (氣缸伸出), above → CLOSE (氣缸縮回)
//   // 根據 SRDF: open = -0.6, close = 0.15
//   // 取中間值 -0.225 作為閾值
//   double threshold_ = -0.225;

//   // 記住目前氣缸是否打開
//   bool is_open_ = false;

//   // GPIO 控制相關
//   std::string gpio_pin_;  // 或者串口命令
//   // 可以根據實際硬體選擇：
//   //   - 直接操作 Linux GPIO (sysfs / libgpiod)
//   //   - 透過 Arduino 串口送 ON/OFF 指令
//   //   - 透過 relay module 控制

//   void set_cylinder(bool open);
// };

// }  // namespace ar4_hardware_interface
