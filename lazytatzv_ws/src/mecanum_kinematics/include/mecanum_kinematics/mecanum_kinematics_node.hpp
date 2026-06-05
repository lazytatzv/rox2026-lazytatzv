#ifndef MECANUM_KINEMATICS__MECANUM_KINEMATICS_NODE_HPP_
#define MECANUM_KINEMATICS__MECANUM_KINEMATICS_NODE_HPP_

#include <memory>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_interfaces/msg/wheel_speeds.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace mecanum_kinematics {

class MecanumKinematicsNode : public rclcpp::Node {
 public:
  explicit MecanumKinematicsNode(const rclcpp::NodeOptions& options);

 private:
  void command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr twist_message);
  rcl_interfaces::msg::SetParametersResult on_parameter_event(
    const std::vector<rclcpp::Parameter>& params);
    
  void declare_parameters();

  double half_length_;
  double half_width_;
  double wheel_radius_;

  std::string topic_cmd_vel_;
  std::string topic_wheel_speeds_;

  rclcpp::Publisher<robot_interfaces::msg::WheelSpeeds>::SharedPtr publisher_wheel_speeds_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_command_velocity_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace mecanum_kinematics

#endif  // MECANUM_KINEMATICS__MECANUM_KINEMATICS_NODE_HPP_
