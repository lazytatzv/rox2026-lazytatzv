#ifndef AT_MOTOR_DRIVER__AT_MOTOR_DRIVER_NODE_HPP_
#define AT_MOTOR_DRIVER__AT_MOTOR_DRIVER_NODE_HPP_

#include <memory>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/wheel_speeds.hpp"
#include "at_motor_driver/at_protocol.hpp"

namespace at_motor_driver {

class AtMotorDriverNode : public rclcpp::Node {
 public:
  explicit AtMotorDriverNode(const rclcpp::NodeOptions& options);
  ~AtMotorDriverNode();

 private:
  void send_velocity_at_command(at_protocol::MotorAddress motor_addr, uint16_t at_value);
  void send_motor_enable_at_commands();
  void wheel_velocities_callback(const robot_interfaces::msg::WheelSpeeds::SharedPtr message);

  int serial_port_file_descriptor_{-1};
  int max_at_command_delta_;
  rclcpp::Subscription<robot_interfaces::msg::WheelSpeeds>::SharedPtr subscription_wheel_velocities_;
};

}  // namespace at_motor_driver

#endif  // AT_MOTOR_DRIVER__AT_MOTOR_DRIVER_NODE_HPP_
