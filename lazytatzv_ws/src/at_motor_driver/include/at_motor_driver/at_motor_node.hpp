#ifndef AT_MOTOR_DRIVER__AT_MOTOR_NODE_HPP_
#define AT_MOTOR_DRIVER__AT_MOTOR_NODE_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "robot_interfaces/msg/at_frame.hpp"
#include "at_motor_driver/at_protocol.hpp"

namespace at_motor_driver {

class AtMotorNode : public rclcpp::Node {
 public:
  explicit AtMotorNode(const rclcpp::NodeOptions& options);

 private:
  void velocity_callback(const std_msgs::msg::Float64::SharedPtr message);
  void send_enable_command();

  uint8_t motor_id_;
  bool invert_direction_;
  int max_at_command_delta_;
  
  rclcpp::Publisher<robot_interfaces::msg::AtFrame>::SharedPtr publisher_at_frames_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_velocity_;
  rclcpp::TimerBase::SharedPtr enable_timer_;
};

}  // namespace at_motor_driver

#endif  // AT_MOTOR_DRIVER__AT_MOTOR_NODE_HPP_
