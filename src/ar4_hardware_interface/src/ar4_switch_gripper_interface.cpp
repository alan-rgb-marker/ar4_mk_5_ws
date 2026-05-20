#include <ar4_hardware_interface/ar4_switch_gripper_interface.hpp>
#include <sstream>

namespace ar4_hardware_interface {

hardware_interface::CallbackReturn ARSwitchGripperHWInterface::on_init(const hardware_interface::HardwareInfo& info) {
  RCLCPP_INFO(logger_, "Initializing hardware interface...");

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = info;

  std::string serial_port = info_.hardware_parameters.at("serial_port");
  int baud_rate = 115200;
  bool success = driver_.init(serial_port, baud_rate);
  if (!success) {
    RCLCPP_WARN(logger_, "Failed to init Arduino driver, but continuing with hardware interface initialization");
    // Continue anyway to allow hardware interface to load
  }

  // Initialize vectors for joint states
  positions_.resize(info_.joints.size(), 0.0);
  velocities_.resize(info_.joints.size(), 1.0);
  position_command_ = 0.0;

  RCLCPP_INFO(logger_, "YES YES YES");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ARSwitchGripperHWInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Activating hardware interface...");

  // position_ = 0.0;
  // position_command_ = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ARSwitchGripperHWInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Deactivating hardware interface...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ARSwitchGripperHWInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocities_[i]);
  }

  RCLCPP_INFO(logger_, "Debug: Exporting state interfaces for %zu joints", info_.joints.size());
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ARSwitchGripperHWInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface("grip_to_base1", hardware_interface::HW_IF_POSITION, &position_command_));
  return command_interfaces;
}

hardware_interface::return_type ARSwitchGripperHWInterface::read(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  // RCLCPP_INFO(logger_, "Reading gripper state from hardware...");

  bool state = driver_.getGripperState();
  double j1_pos = (state == true) ? 0.4886 : 0.01;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    // 三隻夾爪其中一隻應該要反向
    // if (info_.joints[i].name == "grip_to_base2") {
      // positions_[i] = -j1_pos;
    // } else {
      positions_[i] = j1_pos;
    // }
    velocities_[i] = 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ARSwitchGripperHWInterface::write(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  // RCLCPP_INFO(logger_, "Writing gripper command to hardware...");
  // The named targets in SRDF are: open = 1.0, close = 0.0.
  // Use the actual target value rather than assuming >0 means open.
  // bool should_open = position_command_ > 0.0;
  static std::string gripper_cmd_tmp = "";
  // 使用 0.1 作為閾值，避免浮點數誤差導致無法觸發 close
  std::string gripper_cmd = position_command_ > 0.1 ? "ONX14\n" : "OFX14\n";

  if (gripper_cmd == gripper_cmd_tmp) {
    return hardware_interface::return_type::OK; // 狀態沒變，直接返回，不阻塞
  }

  gripper_cmd_tmp = gripper_cmd;
  bool success = driver_.writePosition(position_command_);
  if (!success) {
    RCLCPP_ERROR(logger_, "Failed to write gripper command to hardware");
  }

  std::string logInfo = "Gripper Cmd: " + gripper_cmd;
  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 500, logInfo.c_str());
  return hardware_interface::return_type::OK;
}

}  // namespace ar4_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ar4_hardware_interface::ARSwitchGripperHWInterface, hardware_interface::SystemInterface)
