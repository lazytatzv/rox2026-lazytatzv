#include "at_motor_driver/at_bus_gateway.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include "rclcpp_components/register_node_macro.hpp"

namespace at_motor_driver {

AtBusGateway::AtBusGateway(const rclcpp::NodeOptions& options) 
: Node("at_bus_gateway", options) {
  this->declare_parameter("serial_port", "/dev/ttyUSB1");
  this->declare_parameter("baud_rate", 921600);

  std::string port_path = this->get_parameter("serial_port").as_string();
  int baud_rate = this->get_parameter("baud_rate").as_int();

  serial_port_file_descriptor_ = open(port_path.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_port_file_descriptor_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_path.c_str());
    throw std::runtime_error("Serial port opening failed in Gateway");
  }

  struct termios tty_config;
  tcgetattr(serial_port_file_descriptor_, &tty_config);
  
  speed_t speed = B921600; // Default
  if (baud_rate == 115200) speed = B115200;
  
  cfsetospeed(&tty_config, speed);
  cfsetispeed(&tty_config, speed);
  
  tty_config.c_cflag = (tty_config.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD;
  tty_config.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK);
  tty_config.c_lflag = 0;
  tty_config.c_oflag = 0;
  tcsetattr(serial_port_file_descriptor_, TCSANOW, &tty_config);

  subscription_at_frames_ = this->create_subscription<robot_interfaces::msg::AtFrame>(
    "/at_bus/tx_queue", 100, std::bind(&AtBusGateway::at_frame_callback, this, std::placeholders::_1));
    
  RCLCPP_INFO(this->get_logger(), "AT Bus Gateway started on %s", port_path.c_str());
}

AtBusGateway::~AtBusGateway() {
  if (serial_port_file_descriptor_ >= 0) {
    close(serial_port_file_descriptor_);
  }
}

void AtBusGateway::at_frame_callback(const robot_interfaces::msg::AtFrame::SharedPtr message) {
  if (serial_port_file_descriptor_ < 0) return;
  
  ssize_t written = write(serial_port_file_descriptor_, message->frame_data.data(), message->frame_data.size());
  if (written < 0) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Write to serial failed");
  }
}

}  // namespace at_motor_driver

RCLCPP_COMPONENTS_REGISTER_NODE(at_motor_driver::AtBusGateway)
