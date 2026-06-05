#ifndef BASE_TELEOP__BASE_TELEOP_NODE_HPP_
#define BASE_TELEOP__BASE_TELEOP_NODE_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace base_teleop {

class BaseTeleopNode : public rclcpp::Node {
 public:
  // prevent implicit conversion
  explicit BaseTeleopNode(const rclcpp::NodeOptions& options);

 private:
  void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr joystick_message);
  rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(
      const std::vector<rclcpp::Parameter>& parameters);

  // init parameters
  void declare_parameters();
  void cache_parameters();

  // Publishers and subscriptions
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_command_velocity_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_joystick_;

  // Cached parameters (axis indices)
  int axis_forward_backward_ = 1;
  int axis_left_right_ = 0;
  int axis_yaw_ = 2;
  int axis_deadman_translation_ = 5;
  int axis_deadman_rotation_ = 4;

  // Cached parameters (scaling)
  double scale_linear_velocity_ = 1.0;
  double scale_angular_velocity_ = 1.0;

  // Cached parameters (topics)
  std::string topic_joy_;
  std::string topic_cmd_vel_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace base_teleop

#endif  // BASE_TELEOP__BASE_TELEOP_NODE_HPP_
