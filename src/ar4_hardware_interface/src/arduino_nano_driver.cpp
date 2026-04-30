#include "ar4_hardware_interface/arduino_nano_driver.hpp"

#define FW_VERSION "0.0.1"

namespace ar4_hardware_interface {

bool ArduinoNanoDriver::init(std::string port, int baudrate) {
  // @TODO read version from config
  version_ = FW_VERSION;

  // establish connection with arduino nano board
  boost::system::error_code ec;
  serial_port_.open(port, ec);

  if (ec) {
    RCLCPP_WARN(logger_, "Failed to connect to serial port %s", port.c_str());
    return false;
  } else {
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(static_cast<uint32_t>(baudrate)));
    // serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
    // serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    // serial_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
  }

  RCLCPP_INFO(logger_, "Waiting for response from Arduino Nano on port %s", port.c_str());
  std::this_thread::sleep_for(std::chrono::seconds(2));
  RCLCPP_INFO(logger_, "Successfully initialised driver on port %s", port.c_str());
  return true;
}

ArduinoNanoDriver::ArduinoNanoDriver() : serial_port_(io_service_) {}

std::string ArduinoNanoDriver::sendCommand(std::string outMsg) {
  std::string errTransmit = "";
  if (!transmit(outMsg, errTransmit)) {
    RCLCPP_ERROR(logger_, "Failed to transmit message: %s", errTransmit.c_str());
    return "";
  }

  // std::string inMsg;
  // receive(inMsg);
  // RCLCPP_INFO(logger_, "Received message: %s", inMsg.c_str());
  // return inMsg;
  return "";
}

bool ArduinoNanoDriver::transmit(std::string msg, std::string& err) {
  boost::system::error_code ec;
  const auto sendBuffer = boost::asio::buffer(msg.c_str(), msg.size());

  boost::asio::write(serial_port_, sendBuffer, ec);

  if (!ec) {
    return true;
  } else {
    err = "Error in transmit";
    return false;
  }
}

void ArduinoNanoDriver::receive(std::string& inMsg) {
  char c;
  std::string msg = "";
  bool eol = false;
  while (!eol) {
    boost::asio::read(serial_port_, boost::asio::buffer(&c, 1));
    switch (c) {
      case '\r':
        break;
      case '\n':
        eol = true;
        break;
      default:
        msg += c;
    }
  }
  inMsg = msg;
}

bool ArduinoNanoDriver::checkInit(std::string msg) {
  if (msg == version_) {
    return true;
  } else {
    RCLCPP_ERROR(logger_, "Firmware version mismatch %s vs. %s", msg.c_str(), version_.c_str());
    return false;
  }
}

bool ArduinoNanoDriver::getPosition(int& position) {
  std::string reply = sendCommand("SP0\n");
  if (reply == "") {
    RCLCPP_ERROR(logger_, "Failed to get position");
    return false;
  }
  position = std::stoi(reply);
  return true;
}

bool ArduinoNanoDriver::writePosition(double position) {
  std::string cmd = position < -0.2 ? "ON" : "OF";
  std::string msg = cmd + "X14\n";
  gripper_position_ = position;

  // std::string msg = "SV0P" + std::to_string(static_cast<int>(position)) + "\n";
  // std::string reply = sendCommand(msg);
  sendCommand(msg);
  // if (reply != "Done") {
  //   RCLCPP_ERROR(logger_, "Failed to write position %f", position);
  //   return false;
  // }
  RCLCPP_INFO(logger_, "命令： %s", msg.c_str());
  return true;
}

// 寫一份讀取夾爪狀態的程式 命令是RD
// bool ArduinoNanoDriver::getGripperState() {
//   std::string reply = sendCommand("RD\n");
//   if (reply == "1" )
//   {
//     return true;
//   }
//   else if (reply == "0")
//   {
//     return false;
//   }
//   else
//   {
//     RCLCPP_ERROR(logger_, "Invalid gripper state received: %s", reply.c_str());
//     return false;
//   }
// }
bool ArduinoNanoDriver::getGripperState() {
  RCLCPP_INFO(logger_, "gripper_position_: %f", gripper_position_);
  return gripper_position_ < -0.2 ? true : false;
}

}  // namespace ar4_hardware_interface
