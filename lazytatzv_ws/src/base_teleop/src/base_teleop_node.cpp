#include "base_teleop/base_teleop_node.hpp"
#include <algorithm>
#include "rclcpp_components/register_node_macro.hpp"

/*
  ROS uses a right-handed coordinate system.

  X-axis: forward (positive front)
  Y-axis: Left (positive left)
  Z-axis: Up (positive up)

  Yaw: rotation around Z-axis (positive CCW when looking from above)

*/

namespace base_teleop {

BaseTeleopNode::BaseTeleopNode(const rclcpp::NodeOptions& options) 
: Node("base_teleop_node", options) {
  
  // Declare params with defaults
  declare_parameters();
  cache_parameters();

  // Publisher with SystemDefaultsQoS for chat-like semantics
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
  publisher_command_velocity_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_cmd_vel_, qos);
  
  // Subscription with SystemDefaultsQoS
  subscription_joystick_ = this->create_subscription<sensor_msgs::msg::Joy>(
    topic_joy_, qos, std::bind(&BaseTeleopNode::joystick_callback, this, std::placeholders::_1));

  // Register parameter callback for runtime updates
  this->add_on_set_parameters_callback(
      std::bind(&BaseTeleopNode::on_set_parameters_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "BaseTeleopNode initialized: joy='%s' cmd_vel='%s'", 
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
  topic_joy_ = this->get_parameter("topic_joy").as_string();
  topic_cmd_vel_ = this->get_parameter("topic_cmd_vel").as_string();
}

rcl_interfaces::msg::SetParametersResult BaseTeleopNode::on_set_parameters_callback(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool recreate_required = false;

  for (const auto& param : parameters) {
    const auto& name = param.get_name();
    if (name == "joy_axis_forward_backward" ||
        name == "joy_axis_left_right" ||
        name == "joy_axis_yaw" ||
        name == "joy_axis_deadman_translation" ||
        name == "joy_axis_deadman_rotation") {
      if (param.as_int() < 0) {
        result.successful = false;
        result.reason = "Axis index must be >= 0";
        return result;
      }
    } else if (name == "topic_joy" || name == "topic_cmd_vel") {
      recreate_required = true;
    }
  }

  if (result.successful) {
    cache_parameters();
    if (recreate_required) {
      publisher_command_velocity_.reset();
      subscription_joystick_.reset();

      rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
      publisher_command_velocity_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_cmd_vel_, qos);
      subscription_joystick_ = this->create_subscription<sensor_msgs::msg::Joy>(
        topic_joy_, qos, std::bind(&BaseTeleopNode::joystick_callback, this, std::placeholders::_1));
      
      RCLCPP_INFO(this->get_logger(), "Topics updated: joy='%s' cmd_vel='%s'", 
        topic_joy_.c_str(), topic_cmd_vel_.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Parameters updated successfully");
  }
  return result;
}

void BaseTeleopNode::joystick_callback(const sensor_msgs::msg::Joy::SharedPtr joystick_message) {
  // Robot Frame Twist for mecanum
  auto twist_command = std::make_unique<geometry_msgs::msg::Twist>();

  // Use cached parameters (zero-cost access in hot path)
  if (joystick_message->axes.size() > static_cast<size_t>(std::max({
      axis_forward_backward_, axis_left_right_, axis_yaw_, 
      axis_deadman_translation_, axis_deadman_rotation_}))) 
  {
    bool is_translation_enabled = std::abs(joystick_message->axes[axis_deadman_translation_]) > 0.5;
    bool is_rotation_enabled = std::abs(joystick_message->axes[axis_deadman_rotation_]) > 0.5;

    if (is_translation_enabled) {
      twist_command->linear.x = joystick_message->axes[axis_forward_backward_] * 
                                scale_linear_velocity_;
      twist_command->linear.y = joystick_message->axes[axis_left_right_] * 
                                scale_linear_velocity_;
    }
    if (is_rotation_enabled) {
      twist_command->angular.z = joystick_message->axes[axis_yaw_] * 
                                 scale_angular_velocity_;
    }
  }
  publisher_command_velocity_->publish(std::move(twist_command));
}

}  // namespace base_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(base_teleop::BaseTeleopNode)
