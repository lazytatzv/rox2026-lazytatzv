#ifndef MECANUM_KINEMATICS__WHEEL_SPEEDS_DISPATCHER_HPP_
#define MECANUM_KINEMATICS__WHEEL_SPEEDS_DISPATCHER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "robot_interfaces/msg/wheel_speeds.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace mecanum_kinematics {

class WheelSpeedsDispatcher : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit WheelSpeedsDispatcher(const rclcpp::NodeOptions & options);

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
  void wheel_speeds_callback(const robot_interfaces::msg::WheelSpeeds::SharedPtr msg);
  void declare_parameters();

  std::string front_left_topic_;
  std::string front_right_topic_;
  std::string rear_left_topic_;
  std::string rear_right_topic_;

  rclcpp::Subscription<robot_interfaces::msg::WheelSpeeds>::SharedPtr subscription_;
  
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_fl_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_fr_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_rl_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_rr_;
  
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace mecanum_kinematics

#endif  // MECANUM_KINEMATICS__WHEEL_SPEEDS_DISPATCHER_HPP_
