#include "base_teleop/base_teleop_node.hpp"
#include <algorithm>
#include <chrono>
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace base_teleop {

BaseTeleopNode::BaseTeleopNode(const rclcpp::NodeOptions& options) 
: Node("base_teleop_node", options) {
  
  declare_parameters();
  cache_parameters();

  // Use SensorDataQoS for Joy and SystemDefaults for cmd_vel
  publisher_command_velocity_ = this->create_publisher<geometry_msgs::msg::Twist>(
      topic_cmd_vel_, rclcpp::SystemDefaultsQoS());
  
  subscription_joystick_ = this->create_subscription<sensor_msgs::msg::Joy>(
    topic_joy_, rclcpp::SensorDataQoS(), 
    std::bind(&BaseTeleopNode::joystick_callback, this, std::placeholders::_1));

  // High-frequency control loop for smoothing (50Hz)
  timer_ = this->create_wall_timer(20ms, std::bind(&BaseTeleopNode::timer_callback, this));

  parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&BaseTeleopNode::on_set_parameters_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "BaseTeleopNode [PRO] initialized: joy='%s' cmd_vel='%s'", 
    topic_joy_.c_str(), topic_cmd_vel_.c_str());
}

void BaseTeleopNode::declare_parameters() {
  this->declare_parameter("joy_axis_forward_backward", 1);
  this->declare_parameter("joy_axis_left_right", 0);
  this->declare_parameter("joy_axis_yaw", 2);
  this->declare_parameter("joy_axis_deadman_translation", 5);
  this->declare_parameter("joy_axis_deadman_rotation", 4);
  this->declare_parameter("scale_linear_velocity", 1.0);
  this->declare_parameter("scale_angular_velocity", 1.0);
  this->declare_parameter("smoothing_factor", 0.3);
  this->declare_parameter("topic_joy", "joy");
  this->declare_parameter("topic_cmd_vel", "cmd_vel");
}

void BaseTeleopNode::cache_parameters() {
  axis_forward_backward_ = this->get_parameter("joy_axis_forward_backward").as_int();
  axis_left_right_ = this->get_parameter("joy_axis_left_right").as_int();
  axis_yaw_ = this->get_parameter("joy_axis_yaw").as_int();
  axis_deadman_translation_ = this->get_parameter("joy_axis_deadman_translation").as_int();
  axis_deadman_rotation_ = this->get_parameter("joy_axis_deadman_rotation").as_int();
  scale_linear_velocity_ = this->get_parameter("scale_linear_velocity").as_double();
  scale_angular_velocity_ = this->get_parameter("scale_angular_velocity").as_double();
  smoothing_factor_ = std::clamp(this->get_parameter("smoothing_factor").as_double(), 0.01, 1.0);
  topic_joy_ = this->get_parameter("topic_joy").as_string();
  topic_cmd_vel_ = this->get_parameter("topic_cmd_vel").as_string();
}

void BaseTeleopNode::timer_callback() {
  // Apply Smoothing (Exponential Moving Average)
  auto smooth = [this](double current, double target) {
    return current + smoothing_factor_ * (target - current);
  };

  current_twist_.linear.x = smooth(current_twist_.linear.x, target_twist_.linear.x);
  current_twist_.linear.y = smooth(current_twist_.linear.y, target_twist_.linear.y);
  current_twist_.angular.z = smooth(current_twist_.angular.z, target_twist_.angular.z);

  // Stop completely if very close to zero to avoid micro-creeping
  auto apply_deadband = [](double val) { return (std::abs(val) < 0.001) ? 0.0 : val; };
  current_twist_.linear.x = apply_deadband(current_twist_.linear.x);
  current_twist_.linear.y = apply_deadband(current_twist_.linear.y);
  current_twist_.angular.z = apply_deadband(current_twist_.angular.z);

  // Only publish if moving OR if we just stopped
  static bool was_moving = false;
  bool is_moving = (current_twist_.linear.x != 0.0 || current_twist_.linear.y != 0.0 || current_twist_.angular.z != 0.0);
  
  if (is_moving || was_moving) {
    auto msg = std::make_unique<geometry_msgs::msg::Twist>(current_twist_);
    publisher_command_velocity_->publish(std::move(msg));
  }
  was_moving = is_moving;
}

void BaseTeleopNode::joystick_callback(const sensor_msgs::msg::Joy::SharedPtr joystick_message) {
  size_t required_axes = static_cast<size_t>(std::max({
      axis_forward_backward_, axis_left_right_, axis_yaw_, 
      axis_deadman_translation_, axis_deadman_rotation_}));

  if (joystick_message->axes.size() <= required_axes) return;

  bool is_translation_enabled = std::abs(joystick_message->axes[axis_deadman_translation_]) > 0.5;
  bool is_rotation_enabled = std::abs(joystick_message->axes[axis_deadman_rotation_]) > 0.5;

  target_twist_.linear.x = is_translation_enabled ? (joystick_message->axes[axis_forward_backward_] * scale_linear_velocity_) : 0.0;
  target_twist_.linear.y = is_translation_enabled ? (joystick_message->axes[axis_left_right_] * scale_linear_velocity_) : 0.0;
  target_twist_.angular.z = is_rotation_enabled ? (joystick_message->axes[axis_yaw_] * scale_angular_velocity_) : 0.0;
}

rcl_interfaces::msg::SetParametersResult BaseTeleopNode::on_set_parameters_callback(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto& param : parameters) {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER && param.as_int() < 0) {
      result.successful = false;
      result.reason = "Axis index must be >= 0";
      return result;
    }
  }
  cache_parameters();
  return result;
}

}  // namespace base_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(base_teleop::BaseTeleopNode)
