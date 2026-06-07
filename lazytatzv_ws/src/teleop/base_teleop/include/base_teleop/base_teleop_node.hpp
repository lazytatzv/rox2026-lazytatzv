#ifndef BASE_TELEOP__BASE_TELEOP_NODE_HPP_
#define BASE_TELEOP__BASE_TELEOP_NODE_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

namespace base_teleop {

class BaseTeleopNode : public rclcpp::Node {
 public:
  explicit BaseTeleopNode(const rclcpp::NodeOptions& options);

 private:
  void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr joystick_message);
  void declare_parameters();
  void cache_parameters();
  void timer_callback();

  // Publishers and subscriptions
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_command_velocity_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_stop_lock_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_joystick_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  geometry_msgs::msg::Twist current_twist_;
  geometry_msgs::msg::Twist target_twist_;
  bool joy_mode_active_ = false;

  // Cached parameters (axis indices)
  int axis_forward_backward_ = 1;
  int axis_left_right_ = 0;
  int axis_yaw_ = 2;
  int axis_deadman_translation_ = 5;
  int axis_deadman_rotation_ = 4;
  int button_software_stop_ = 15;
  int button_joy_mode_on_ = 8;

  // Cached parameters (scaling)
  double scale_linear_velocity_ = 1.0;
  double scale_angular_velocity_ = 1.0;
  double smoothing_factor_ = 0.3;

  // Cached parameters (topics)
  std::string topic_joy_;
  std::string topic_cmd_vel_;
  std::string topic_stop_lock_;
};

}  // namespace base_teleop

#endif  // BASE_TELEOP__BASE_TELEOP_NODE_HPP_
