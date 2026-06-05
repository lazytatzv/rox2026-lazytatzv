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
  publisher_command_velocity_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);
  
  // Subscription with SystemDefaultsQoS
  subscription_joystick_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", qos, std::bind(&BaseTeleopNode::joystick_callback, this, std::placeholders::_1));

  // Register parameter callback for runtime updates
  this->add_on_set_parameters_callback(
      std::bind(&BaseTeleopNode::on_set_parameters_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "BaseTeleopNode initialized");
}

void BaseTeleopNode::declare_parameters() {
  this->declare_parameter("joy_axis_forward_backward", 1);
  this->declare_parameter("joy_axis_left_right", 0);
  this->declare_parameter("joy_axis_yaw", 2);
  this->declare_parameter("joy_axis_deadman_translation", 5);
  this->declare_parameter("joy_axis_deadman_rotation", 4);
  this->declare_parameter("scale_linear_velocity", 1.0);
  this->declare_parameter("scale_angular_velocity", 1.0);
}

void BaseTeleopNode::cache_parameters() {
  axis_forward_backward_ = this->get_parameter("joy_axis_forward_backward").as_int();
  axis_left_right_ = this->get_parameter("joy_axis_left_right").as_int();
  axis_yaw_ = this->get_parameter("joy_axis_yaw").as_int();
  axis_deadman_translation_ = this->get_parameter("joy_axis_deadman_translation").as_int();
  axis_deadman_rotation_ = this->get_parameter("joy_axis_deadman_rotation").as_int();
  scale_linear_velocity_ = this->get_parameter("scale_linear_velocity").as_double();
  scale_angular_velocity_ = this->get_parameter("scale_angular_velocity").as_double();
}

rcl_interfaces::msg::SetParametersResult BaseTeleopNode::on_set_parameters_callback(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& param : parameters) {
    if (param.get_name() == "joy_axis_forward_backward" ||
        param.get_name() == "joy_axis_left_right" ||
        param.get_name() == "joy_axis_yaw" ||
        param.get_name() == "joy_axis_deadman_translation" ||
        param.get_name() == "joy_axis_deadman_rotation") {
      // Validate axis index >= 0
      if (param.as_int() < 0) {
        result.successful = false;
        result.reason = "Axis index must be >= 0";
        return result;
      }
    } else if (param.get_name() == "scale_linear_velocity" ||
               param.get_name() == "scale_angular_velocity") {
      // Scale factors can be any value (including negative for reversing)
      // but warn if very large
      if (std::abs(param.as_double()) > 10.0) {
        RCLCPP_WARN(this->get_logger(), 
            "Parameter %s has large scale value: %f", 
            param.get_name().c_str(), param.as_double());
      }
    }
  }

  // Cache all parameters if successful
  if (result.successful) {
    cache_parameters();
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
    bool is_translation_enabled = joystick_message->axes[axis_deadman_translation_] < 0.9;
    bool is_rotation_enabled = joystick_message->axes[axis_deadman_rotation_] < 0.9;

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
