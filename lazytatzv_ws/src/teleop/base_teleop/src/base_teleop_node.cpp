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

  publisher_command_velocity_ = this->create_publisher<geometry_msgs::msg::Twist>(
      topic_cmd_vel_, rclcpp::SystemDefaultsQoS());

  publisher_stop_lock_ = this->create_publisher<std_msgs::msg::Bool>(
      topic_stop_lock_, rclcpp::SystemDefaultsQoS());
  
  subscription_joystick_ = this->create_subscription<sensor_msgs::msg::Joy>(
    topic_joy_, rclcpp::SensorDataQoS(), 
    std::bind(&BaseTeleopNode::joystick_callback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(20ms, std::bind(&BaseTeleopNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "BaseTeleopNode initialized: joy='%s' cmd_vel='%s'", 
    topic_joy_.c_str(), topic_cmd_vel_.c_str());
}

void BaseTeleopNode::declare_parameters() {
  this->declare_parameter("joy_axis_forward_backward", 1);
  this->declare_parameter("joy_axis_left_right", 0);
  this->declare_parameter("joy_axis_yaw", 2);
  this->declare_parameter("joy_axis_deadman_translation", 5);
  this->declare_parameter("joy_axis_deadman_rotation", 4);
  this->declare_parameter("joy_button_software_stop", 15);
  this->declare_parameter("joy_button_joy_mode_on", 8);
  this->declare_parameter("scale_linear_velocity", 1.0);
  this->declare_parameter("scale_angular_velocity", 1.0);
  this->declare_parameter("smoothing_factor", 0.3);
  this->declare_parameter("topic_joy", "joy");
  this->declare_parameter("topic_cmd_vel", "cmd_vel");
  this->declare_parameter("topic_stop_lock", "stop_lock");
}

void BaseTeleopNode::cache_parameters() {
  axis_forward_backward_ = this->get_parameter("joy_axis_forward_backward").as_int();
  axis_left_right_ = this->get_parameter("joy_axis_left_right").as_int();
  axis_yaw_ = this->get_parameter("joy_axis_yaw").as_int();
  axis_deadman_translation_ = this->get_parameter("joy_axis_deadman_translation").as_int();
  axis_deadman_rotation_ = this->get_parameter("joy_axis_deadman_rotation").as_int();
  button_software_stop_ = this->get_parameter("joy_button_software_stop").as_int();
  button_joy_mode_on_ = this->get_parameter("joy_button_joy_mode_on").as_int();
  scale_linear_velocity_ = this->get_parameter("scale_linear_velocity").as_double();
  scale_angular_velocity_ = this->get_parameter("scale_angular_velocity").as_double();
  smoothing_factor_ = std::clamp(this->get_parameter("smoothing_factor").as_double(), 0.01, 1.0);
  topic_joy_ = this->get_parameter("topic_joy").as_string();
  topic_cmd_vel_ = this->get_parameter("topic_cmd_vel").as_string();
  topic_stop_lock_ = this->get_parameter("topic_stop_lock").as_string();
}

void BaseTeleopNode::timer_callback() {
  auto smooth = [this](double current, double target) {
    return current + smoothing_factor_ * (target - current);
  };

  current_twist_.linear.x = smooth(current_twist_.linear.x, target_twist_.linear.x);
  current_twist_.linear.y = smooth(current_twist_.linear.y, target_twist_.linear.y);
  current_twist_.angular.z = smooth(current_twist_.angular.z, target_twist_.angular.z);

  auto apply_deadband = [](double val) { return (std::abs(val) < 0.001) ? 0.0 : val; };
  current_twist_.linear.x = apply_deadband(current_twist_.linear.x);
  current_twist_.linear.y = apply_deadband(current_twist_.linear.y);
  current_twist_.angular.z = apply_deadband(current_twist_.angular.z);

  static bool was_moving = false;
  bool is_moving = (current_twist_.linear.x != 0.0 || current_twist_.linear.y != 0.0 || current_twist_.angular.z != 0.0);
  
  if (is_moving || was_moving) {
    auto msg = std::make_unique<geometry_msgs::msg::Twist>(current_twist_);
    publisher_command_velocity_->publish(std::move(msg));
  }
  was_moving = is_moving;
}

void BaseTeleopNode::joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  size_t required_buttons = static_cast<size_t>(std::max(button_software_stop_, button_joy_mode_on_));
  if (msg->buttons.size() <= required_buttons) return;

  // 1. STOP (Touchpad)
  static bool last_stop_state = false;
  if (msg->buttons[button_software_stop_] == 1 && !last_stop_state) {
    joy_mode_active_ = false;
    auto lock = std::make_unique<std_msgs::msg::Bool>();
    lock->data = true;
    publisher_stop_lock_->publish(std::move(lock));
    RCLCPP_WARN(get_logger(), "SYSTEM LOCKED");
  }
  last_stop_state = (msg->buttons[button_software_stop_] == 1);

  // 2. JOY (Select)
  static bool last_joy_state = false;
  if (msg->buttons[button_joy_mode_on_] == 1 && !last_joy_state) {
    joy_mode_active_ = true;
    auto lock = std::make_unique<std_msgs::msg::Bool>();
    lock->data = false;
    publisher_stop_lock_->publish(std::move(lock));
    RCLCPP_INFO(get_logger(), "SYSTEM ARMED");
  }
  last_joy_state = (msg->buttons[button_joy_mode_on_] == 1);

  // 3. PROCESSING
  if (!joy_mode_active_) {
    target_twist_.linear.x = 0.0; target_twist_.linear.y = 0.0; target_twist_.angular.z = 0.0;
    return;
  }

  size_t required_axes = static_cast<size_t>(std::max({
      axis_forward_backward_, axis_left_right_, axis_yaw_, 
      axis_deadman_translation_, axis_deadman_rotation_}));
  if (msg->axes.size() <= required_axes) return;

  bool deadman = (std::abs(msg->axes[axis_deadman_translation_]) > 0.5 || 
                  std::abs(msg->axes[axis_deadman_rotation_]) > 0.5);

  target_twist_.linear.x = deadman ? (msg->axes[axis_forward_backward_] * scale_linear_velocity_) : 0.0;
  target_twist_.linear.y = deadman ? (msg->axes[axis_left_right_] * scale_linear_velocity_) : 0.0;
  target_twist_.angular.z = deadman ? (msg->axes[axis_yaw_] * scale_angular_velocity_) : 0.0;
}

}  // namespace base_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(base_teleop::BaseTeleopNode)
