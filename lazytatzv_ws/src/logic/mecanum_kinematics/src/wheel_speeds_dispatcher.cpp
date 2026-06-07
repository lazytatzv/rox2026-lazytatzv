// Copyright 2026 Tatsukiyano
#include "mecanum_kinematics/wheel_speeds_dispatcher.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <vector>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace mecanum_kinematics
{

WheelSpeedsDispatcher::WheelSpeedsDispatcher(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("wheel_speeds_dispatcher", options)
{
  declare_parameters();
}

void WheelSpeedsDispatcher::declare_parameters()
{
  this->declare_parameter("front_left_topic", std::string("/motors/front_left/velocity_command"));
  this->declare_parameter("front_right_topic", std::string("/motors/front_right/velocity_command"));
  this->declare_parameter("rear_left_topic", std::string("/motors/rear_left/velocity_command"));
  this->declare_parameter("rear_right_topic", std::string("/motors/rear_right/velocity_command"));
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WheelSpeedsDispatcher::on_configure(const rclcpp_lifecycle::State &)
{
  front_left_topic_ = this->get_parameter("front_left_topic").as_string();
  front_right_topic_ = this->get_parameter("front_right_topic").as_string();
  rear_left_topic_ = this->get_parameter("rear_left_topic").as_string();
  rear_right_topic_ = this->get_parameter("rear_right_topic").as_string();

  subscription_ = this->create_subscription<robot_interfaces::msg::WheelSpeeds>(
    "wheel_speeds",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&WheelSpeedsDispatcher::wheel_speeds_callback, this, std::placeholders::_1));

  pub_fl_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(front_left_topic_,
      rclcpp::SystemDefaultsQoS());
  pub_fr_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(front_right_topic_,
      rclcpp::SystemDefaultsQoS());
  pub_rl_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(rear_left_topic_,
      rclcpp::SystemDefaultsQoS());
  pub_rr_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(rear_right_topic_,
      rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_logger(), "Configured");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WheelSpeedsDispatcher::on_activate(const rclcpp_lifecycle::State &)
{
  pub_fl_->on_activate();
  pub_fr_->on_activate();
  pub_rl_->on_activate();
  pub_rr_->on_activate();
  RCLCPP_INFO(get_logger(), "Activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WheelSpeedsDispatcher::on_deactivate(const rclcpp_lifecycle::State &)
{
  pub_fl_->on_deactivate();
  pub_fr_->on_deactivate();
  pub_rl_->on_deactivate();
  pub_rr_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WheelSpeedsDispatcher::on_cleanup(const rclcpp_lifecycle::State &)
{
  subscription_.reset();
  pub_fl_.reset();
  pub_fr_.reset();
  pub_rl_.reset();
  pub_rr_.reset();
  RCLCPP_INFO(get_logger(), "Cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WheelSpeedsDispatcher::on_shutdown(const rclcpp_lifecycle::State &)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void WheelSpeedsDispatcher::wheel_speeds_callback(
  const robot_interfaces::msg::WheelSpeeds::SharedPtr msg)
{
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {return;}

  auto publish_vec =
    [](const rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr & pub,
    double velocity) {
      auto out = std::make_unique<std_msgs::msg::Float64MultiArray>();
      out->data.push_back(velocity);
      out->data.push_back(1.0); // Dummy current limit
      pub->publish(std::move(out));
    };

  publish_vec(pub_fl_, msg->front_left_velocity);
  publish_vec(pub_fr_, msg->front_right_velocity);
  publish_vec(pub_rl_, msg->rear_left_velocity);
  publish_vec(pub_rr_, msg->rear_right_velocity);
}

}  // namespace mecanum_kinematics

RCLCPP_COMPONENTS_REGISTER_NODE(mecanum_kinematics::WheelSpeedsDispatcher)
