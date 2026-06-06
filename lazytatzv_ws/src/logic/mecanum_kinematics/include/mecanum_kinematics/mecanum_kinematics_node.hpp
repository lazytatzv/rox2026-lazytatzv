#ifndef MECANUM_KINEMATICS__MECANUM_KINEMATICS_NODE_HPP_
#define MECANUM_KINEMATICS__MECANUM_KINEMATICS_NODE_HPP_

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "robot_interfaces/msg/wheel_speeds.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace mecanum_kinematics {

class MecanumKinematicsNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit MecanumKinematicsNode(const rclcpp::NodeOptions& options);

  // Lifecycle Transitions
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

 private:
  void command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr twist_message);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  
  void declare_parameters();
  void update_parameters();

  // Robot Geometry
  double half_length_;
  double half_width_;
  double wheel_radius_;

  // Odometry State
  double x_ = 0.0;
  double y_ = 0.0;
  double th_ = 0.0;
  rclcpp::Time last_time_;
  bool first_odom_ = true;

  std::string topic_cmd_vel_;
  std::string topic_wheel_speeds_;

  // Lifecycle Publishers
  rclcpp_lifecycle::LifecyclePublisher<robot_interfaces::msg::WheelSpeeds>::SharedPtr publisher_wheel_speeds_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom_;
  
  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_command_velocity_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_states_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace mecanum_kinematics

#endif  // MECANUM_KINEMATICS__MECANUM_KINEMATICS_NODE_HPP_
