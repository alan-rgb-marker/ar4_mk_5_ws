#include "ar4_hardware_interface/teensy_driver.hpp"

#include <chrono>
#include <thread>

#define FW_VERSION "0.0.1"

namespace ar4_hardware_interface {

void TeensyDriver::init(std::string port, int baudrate, int num_joints) {
  // @TODO read version from config
  version_ = FW_VERSION;

  // establish connection with teensy board
  boost::system::error_code ec;
  serial_port_.open(port, ec);

  if (ec) {
    RCLCPP_WARN(logger_, "Failed to connect to serial port %s", port.c_str());
    return;
  } else {
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(static_cast<uint32_t>(baudrate)));
    serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    RCLCPP_INFO(logger_, "Successfully connected to serial port %s", port.c_str());
  }

  initialised_ = true;
  RCLCPP_INFO(logger_, "Skipping HO handshake as firmware does not support it.");
  RCLCPP_INFO(logger_, "Successfully initialised driver on port %s", port.c_str());

  // initialise joint and encoder calibration
  num_joints_ = num_joints;
  joint_positions_deg_.resize(num_joints_);
  enc_calibrations_.resize(num_joints_);
}

TeensyDriver::TeensyDriver() : serial_port_(io_service_) {}

void TeensyDriver::setStepperSpeed(std::vector<double>& max_speed, std::vector<double>& max_accel) {
  std::string outMsg = "SS";
  for (int i = 0, charIdx = 0; i < num_joints_; ++i, charIdx += 2) {
    outMsg += 'A' + charIdx;
    outMsg += std::to_string(max_speed[i]);
    outMsg += 'A' + charIdx + 1;
    outMsg += std::to_string(max_accel[i]);
  }
  outMsg += "\n";
  exchange(outMsg);
}

// Update between hardware interface and hardware driver
void TeensyDriver::update(std::vector<double>& pos_commands, std::vector<double>& joint_positions) {
  // For now, just get current positions since Teensy doesn't support direct joint commands
  // TODO: Implement proper joint control or modify Teensy firmware
  // getJointPositions(joint_positions);
  static std::string outMsg_tmp = "";
  //四捨五入
  for (size_t i = 0; i < pos_commands.size(); ++i) {
    pos_commands[i] = (int)(pos_commands[i] * 100) / 100.0;
  } 

  std::string outMsg = "RJ";
  outMsg += "A" + std::to_string(pos_commands[0]);  // J1
  outMsg += "B" + std::to_string(pos_commands[1]);  // J2
  outMsg += "C" + std::to_string(pos_commands[2]);  // J3
  outMsg += "D" + std::to_string(pos_commands[3]);  // J4
  outMsg += "E" + std::to_string(pos_commands[4]);  // J5
  outMsg += "F" + std::to_string(pos_commands[5]);  // J6
  // outMsg += "J70J80J90Sp10Ac10Dc10Rm20W0\n"; 
  outMsg += "G0Sp10Ac10Dc10Rm20W0\n";
  
  
  if(outMsg == outMsg_tmp) {
    RCLCPP_INFO_THROTTLE(logger_, clock_, 500, "same pos: %s", outMsg.c_str());
    return;
  }
  else {
    outMsg_tmp = outMsg;
    RCLCPP_INFO(logger_, "Send msg: %s", outMsg.c_str());
    sendCommand(outMsg_tmp);                 // 發送一次
  }
}

void TeensyDriver::calibrateJoints() {
  // For now, just log that calibration was requested
  // Teensy calibration is typically done through limit switches
  RCLCPP_INFO(logger_, "Joint calibration requested - ensure robot is in calibration position");
  
  // Could send "TL" command to test limit switches, but for now just acknowledge
  std::string outMsg = "TL\n";
  sendCommand(outMsg);
}

void TeensyDriver::getJointPositions(std::vector<double>& joint_positions) {
  // get current joint positions
  std::string msg = "RP\n";
  exchange(msg);
  joint_positions = joint_positions_deg_;
}

// Send specific commands
void TeensyDriver::sendCommand(std::string outMsg) {
  exchange(outMsg);
}

// Send msg to board and collect data
void TeensyDriver::exchange(std::string outMsg) {
  std::string inMsg;
  std::string errTransmit = "";

  std::string aux = "Driver exchange outMsg" + outMsg;
  RCLCPP_INFO_THROTTLE(logger_, clock_, 500, aux.c_str());

  if (!transmit(outMsg, errTransmit)) {
    RCLCPP_ERROR(logger_, "Error in transmit: %s", errTransmit.c_str());
  }

  bool done = false;
  while (!done) {
    receive(inMsg);
    // parse msg
    if (inMsg.length() > 0) {
      if (inMsg[0] == '{') {
        // JSON response from HO command
        checkInit(inMsg);
        done = true;
      } else if (inMsg[0] == 'A') {
        // Position response from RP command
        updateJointPositions(inMsg);
        done = true;
      } else {
        // unknown header
        RCLCPP_WARN(logger_, "Unknown response: %s", inMsg.c_str());
        done = true;
      }
    }
  }
}

bool TeensyDriver::transmit(std::string msg, std::string& err) {
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

void TeensyDriver::receive(std::string& inMsg) {
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

void TeensyDriver::checkInit(std::string msg) {
  // Parse JSON response from HO command
  // For now, just check if we got a valid JSON response
  if (msg.find("DriverModel") != std::string::npos) {
    initialised_ = true;
    RCLCPP_INFO(logger_, "Received valid Teensy identification: %s", msg.c_str());
  } else {
    RCLCPP_ERROR(logger_, "Invalid Teensy identification response: %s", msg.c_str());
  }
}

void TeensyDriver::updateEncoderCalibrations(std::string msg) {
  size_t idx1 = msg.find("A", 2) + 1;
  size_t idx2 = msg.find("B", 2) + 1;
  size_t idx3 = msg.find("C", 2) + 1;
  size_t idx4 = msg.find("D", 2) + 1;
  size_t idx5 = msg.find("E", 2) + 1;
  size_t idx6 = msg.find("F", 2) + 1;
  enc_calibrations_[0] = std::stoi(msg.substr(idx1, idx2 - idx1));
  enc_calibrations_[1] = std::stoi(msg.substr(idx2, idx3 - idx2));
  enc_calibrations_[2] = std::stoi(msg.substr(idx3, idx4 - idx3));
  enc_calibrations_[3] = std::stoi(msg.substr(idx4, idx5 - idx4));
  enc_calibrations_[4] = std::stoi(msg.substr(idx5, idx6 - idx5));
  enc_calibrations_[5] = std::stoi(msg.substr(idx6));

  // @TODO update config file
  RCLCPP_INFO(logger_, "Successfully updated encoder calibrations");
}

void TeensyDriver::updateJointPositions(std::string msg) {
  // Parse format: A{joint1}B{joint2}C{joint3}D{joint4}E{joint5}F{joint6}...
  size_t pos = 0;
  for (int i = 0; i < num_joints_ && pos < msg.length(); ++i) {
    char marker = 'A' + i;
    size_t start = msg.find(marker, pos);
    if (start != std::string::npos) {
      start += 1; // Skip the marker
      size_t end = (i < num_joints_ - 1) ? msg.find(char('A' + i + 1), start) : msg.length();
      if (end != std::string::npos) {
        std::string value = msg.substr(start, end - start);
        try {
          joint_positions_deg_[i] = std::stod(value);
        } catch (const std::exception& e) {
          RCLCPP_WARN(logger_, "Failed to parse joint %d position: %s", i, value.c_str());
          joint_positions_deg_[i] = 0.0;
        }
        pos = end;
      }
    }
  }
}

}  // namespace ar4_hardware_interface
