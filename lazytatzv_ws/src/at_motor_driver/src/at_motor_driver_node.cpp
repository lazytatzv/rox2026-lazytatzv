#include "at_motor_driver/at_motor_driver_node.hpp"
#include <thread>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include "rclcpp_components/register_node_macro.hpp"

namespace at_motor_driver {

using namespace at_protocol;

AtMotorDriverNode::AtMotorDriverNode(const rclcpp::NodeOptions& options) 
: Node("at_motor_driver_node", options) {
  this->declare_parameter("serial_port", "/dev/ttyUSB1");
  this->declare_parameter("max_speed_limit_percentage", 50.0);

  std::string port_path = this->get_parameter("serial_port").as_string();
  double max_speed_percentage = this->get_parameter("max_speed_limit_percentage").as_double();
  
  max_at_command_delta_ = static_cast<int>(NEUTRAL_VELOCITY_VALUE * (max_speed_percentage / 100.0));

  serial_port_file_descriptor_ = open(port_path.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_port_file_descriptor_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_path.c_str());
    throw std::runtime_error("Serial port opening failed");
  }

  struct termios tty_config;
  tcgetattr(serial_port_file_descriptor_, &tty_config);
  cfsetospeed(&tty_config, B921600);
  cfsetispeed(&tty_config, B921600);
  tty_config.c_cflag = (tty_config.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD;
  tty_config.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK);
  tty_config.c_lflag = 0;
  tty_config.c_oflag = 0;
  tcsetattr(serial_port_file_descriptor_, TCSANOW, &tty_config);

  send_motor_enable_at_commands();

  subscription_wheel_velocities_ = this->create_subscription<robot_interfaces::msg::WheelSpeeds>(
    "wheel_speeds", 10, std::bind(&AtMotorDriverNode::wheel_velocities_callback, this, std::placeholders::_1));
}

AtMotorDriverNode::~AtMotorDriverNode() {
  if (serial_port_file_descriptor_ >= 0) {
    close(serial_port_file_descriptor_);
  }
}

void AtMotorDriverNode::send_velocity_at_command(MotorAddress motor_addr, uint16_t at_value) {
  uint8_t direction_flag = (at_value == NEUTRAL_VELOCITY_VALUE) ? DIR_STOP : DIR_ROTATING;
  
  std::vector<uint8_t> packet = {
    FRAME_HEADER_A,
    FRAME_HEADER_T,
    CMD_DATA_STREAMING,
    DEFAULT_SOURCE_ID_HI,
    DEFAULT_SOURCE_ID_LO,
    static_cast<uint8_t>(motor_addr),
    DATA_LEN_8_BYTES,
    SPEED_CMD_INDICATOR,
    REG_ADDR_VELOCITY_CTRL,
    0x00, // Reserved
    0x00, // Reserved
    CTRL_MODE_VELOCITY,
    direction_flag,
    static_cast<uint8_t>((at_value >> 8) & 0xFF),
    static_cast<uint8_t>(at_value & 0xFF),
    FRAME_FOOTER_CR,
    FRAME_FOOTER_LF
  };
  write(serial_port_file_descriptor_, packet.data(), packet.size());
}

void AtMotorDriverNode::send_motor_enable_at_commands() {
  auto build_enable_packet = [](MotorAddress motor_addr) -> std::vector<uint8_t> {
    return {
      FRAME_HEADER_A,
      FRAME_HEADER_T,
      CMD_BASIC_CONFIG,
      DEFAULT_SOURCE_ID_HI,
      DEFAULT_SOURCE_ID_LO,
      static_cast<uint8_t>(motor_addr),
      DATA_LEN_8_BYTES,
      0x00, // Reserved/Length detail
      REG_ADDR_MOTOR_ENABLE,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Data padding
      FRAME_FOOTER_CR,
      FRAME_FOOTER_LF
    };
  };
  
  const std::vector<MotorAddress> motor_addresses = {
    MotorAddress::FRONT_LEFT,
    MotorAddress::FRONT_RIGHT,
    MotorAddress::REAR_LEFT,
    MotorAddress::REAR_RIGHT
  };

  for (auto addr : motor_addresses) {
    auto packet = build_enable_packet(addr);
    write(serial_port_file_descriptor_, packet.data(), packet.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

void AtMotorDriverNode::wheel_velocities_callback(const robot_interfaces::msg::WheelSpeeds::SharedPtr message) {
  auto convert_to_at_value = [&](double normalized_velocity) -> uint16_t {
    int delta = static_cast<int>(std::round(normalized_velocity * max_at_command_delta_));
    return static_cast<uint16_t>(NEUTRAL_VELOCITY_VALUE + delta);
  };

  send_velocity_at_command(MotorAddress::FRONT_LEFT,  convert_to_at_value(message->front_left_velocity));
  send_velocity_at_command(MotorAddress::FRONT_RIGHT, convert_to_at_value(-message->front_right_velocity));
  send_velocity_at_command(MotorAddress::REAR_LEFT,   convert_to_at_value(message->rear_left_velocity));
  send_velocity_at_command(MotorAddress::REAR_RIGHT,  convert_to_at_value(-message->rear_right_velocity));
}

}  // namespace at_motor_driver

RCLCPP_COMPONENTS_REGISTER_NODE(at_motor_driver::AtMotorDriverNode)
