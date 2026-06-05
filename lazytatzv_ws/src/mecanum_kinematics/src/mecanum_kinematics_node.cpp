#include "mecanum_kinematics/mecanum_kinematics_node.hpp"

#include "mecanum_kinematics/kinematics.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <functional>
#include <vector>

namespace mecanum_kinematics {

MecanumKinematicsNode::MecanumKinematicsNode(const rclcpp::NodeOptions & options)
: Node("mecanum_kinematics_node", options)
{
  declare_parameters();

  half_length_ = this->get_parameter("half_length").as_double();
  half_width_ = this->get_parameter("half_width").as_double();
  wheel_radius_ = this->get_parameter("wheel_radius").as_double();

  publisher_wheel_speeds_ = this->create_publisher<robot_interfaces::msg::WheelSpeeds>(
    "wheel_speeds", rclcpp::SystemDefaultsQoS());

  subscription_command_velocity_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MecanumKinematicsNode::command_velocity_callback, this, std::placeholders::_1));

  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&MecanumKinematicsNode::on_parameter_event, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "MecanumKinematicsNode started: half_length=%.3f half_width=%.3f wheel_radius=%.3f",
    half_length_,
    half_width_,
    wheel_radius_);
}

void MecanumKinematicsNode::declare_parameters()
{
  this->declare_parameter("half_length", 0.12);
  this->declare_parameter("half_width", 0.10);
  this->declare_parameter("wheel_radius", 0.05);
}

void MecanumKinematicsNode::command_velocity_callback(
  const geometry_msgs::msg::Twist::SharedPtr twist_message)
{
  const double linear_x_velocity = twist_message->linear.x;
  const double linear_y_velocity = twist_message->linear.y;
  const double angular_z_velocity = twist_message->angular.z;

  const auto wheels = mecanum_kinematics::compute_wheel_speeds(
    linear_x_velocity,
    linear_y_velocity,
    angular_z_velocity,
    half_length_,
    half_width_,
    wheel_radius_);

  auto msg = std::make_unique<robot_interfaces::msg::WheelSpeeds>();
  msg->front_left_velocity = wheels[0];
  msg->front_right_velocity = wheels[1];
  msg->rear_left_velocity = wheels[2];
  msg->rear_right_velocity = wheels[3];

  publisher_wheel_speeds_->publish(std::move(msg));
}

rcl_interfaces::msg::SetParametersResult MecanumKinematicsNode::on_parameter_event(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "ok";

  for (const auto & param : params) {
    if (param.get_name() == "half_length") {
      half_length_ = param.as_double();
    } else if (param.get_name() == "half_width") {
      half_width_ = param.as_double();
    } else if (param.get_name() == "wheel_radius") {
      const double radius = param.as_double();
      if (radius <= 0.0) {
        result.successful = false;
        result.reason = "wheel_radius must be > 0";
        return result;
      }
      wheel_radius_ = radius;
    }
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Updated params: half_length=%.3f half_width=%.3f wheel_radius=%.3f",
    half_length_,
    half_width_,
    wheel_radius_);

  return result;
}

}  // namespace mecanum_kinematics

RCLCPP_COMPONENTS_REGISTER_NODE(mecanum_kinematics::MecanumKinematicsNode)
