#ifndef MOTOR_DRIVER__ROBSTRIDE_MOTOR_NODE_HPP_
#define MOTOR_DRIVER__ROBSTRIDE_MOTOR_NODE_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "robot_interfaces/msg/serial_frame.hpp"
#include "motor_driver/at_protocol.hpp"

namespace motor_driver {

class RobstrideMotorNode : public rclcpp::Node {
 public:
  explicit RobstrideMotorNode(const rclcpp::NodeOptions& options);

 private:
  void velocity_callback(const std_msgs::msg::Float64::SharedPtr message);
  void send_enable_command();

  uint8_t motor_id_;
  bool invert_direction_;
  int max_at_command_delta_;

  std::string topic_tx_queue_;
  std::string topic_target_velocity_;
  
  rclcpp::Publisher<robot_interfaces::msg::SerialFrame>::SharedPtr publisher_serial_frames_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_velocity_;
  rclcpp::TimerBase::SharedPtr enable_timer_;
};

}  // namespace motor_driver

#endif  // MOTOR_DRIVER__ROBSTRIDE_MOTOR_NODE_HPP_
