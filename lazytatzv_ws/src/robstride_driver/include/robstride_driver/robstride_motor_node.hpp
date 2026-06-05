#ifndef ROBSTRIDE_DRIVER__ROBSTRIDE_MOTOR_NODE_HPP_
#define ROBSTRIDE_DRIVER__ROBSTRIDE_MOTOR_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_interfaces/msg/serial_frame.hpp"
#include "robstride_driver/at_protocol.hpp"

namespace robstride_driver {

class RobstrideMotorNode : public rclcpp::Node {
 public:
  explicit RobstrideMotorNode(const rclcpp::NodeOptions& options);

 private:
  void velocity_callback(const std_msgs::msg::Float64MultiArray::SharedPtr message);
  void serial_rx_callback(const robot_interfaces::msg::SerialFrame::SharedPtr message);
  void send_enable_command();
  
  // Scaling utilities
  double uint_to_float(uint16_t value, double low, double high);

  // Parameters
  uint8_t motor_id_;
  bool invert_direction_;
  int max_at_command_delta_;
  std::string topic_tx_queue_;
  std::string topic_rx_queue_;
  std::string topic_target_velocity_;
  std::string joint_name_;

  // Range parameters (Standard EL05 specs from senior's code)
  double pos_min_ = -12.57;
  double pos_max_ = 12.57;
  double vel_min_ = -50.0;
  double vel_max_ = 50.0;
  double tor_min_ = -6.0;
  double tor_max_ = 6.0;

  // ROS components
  rclcpp::Publisher<robot_interfaces::msg::SerialFrame>::SharedPtr publisher_serial_frames_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_joint_state_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_velocity_;
  rclcpp::Subscription<robot_interfaces::msg::SerialFrame>::SharedPtr subscription_serial_rx_;
  
  rclcpp::TimerBase::SharedPtr enable_timer_;
};

}  // namespace robstride_driver

#endif  // ROBSTRIDE_DRIVER__ROBSTRIDE_MOTOR_NODE_HPP_
