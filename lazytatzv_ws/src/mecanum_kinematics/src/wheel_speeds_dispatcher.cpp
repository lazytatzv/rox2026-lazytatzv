#include "mecanum_kinematics/wheel_speeds_dispatcher.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

namespace mecanum_kinematics {

WheelSpeedsDispatcher::WheelSpeedsDispatcher(const rclcpp::NodeOptions & options)
: Node("wheel_speeds_dispatcher", options)
{
  declare_parameters();

  max_wheel_speed_ = this->get_parameter("max_wheel_speed").as_double();
  front_left_topic_ = this->get_parameter("front_left_topic").as_string();
  front_right_topic_ = this->get_parameter("front_right_topic").as_string();
  rear_left_topic_ = this->get_parameter("rear_left_topic").as_string();
  rear_right_topic_ = this->get_parameter("rear_right_topic").as_string();

  subscription_ = this->create_subscription<robot_interfaces::msg::WheelSpeeds>(
    "wheel_speeds",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&WheelSpeedsDispatcher::wheel_speeds_callback, this, std::placeholders::_1));

  pub_fl_ = this->create_publisher<std_msgs::msg::Float64>(front_left_topic_, rclcpp::SystemDefaultsQoS());
  pub_fr_ = this->create_publisher<std_msgs::msg::Float64>(front_right_topic_, rclcpp::SystemDefaultsQoS());
  pub_rl_ = this->create_publisher<std_msgs::msg::Float64>(rear_left_topic_, rclcpp::SystemDefaultsQoS());
  pub_rr_ = this->create_publisher<std_msgs::msg::Float64>(rear_right_topic_, rclcpp::SystemDefaultsQoS());

  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&WheelSpeedsDispatcher::on_parameter_event, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "WheelSpeedsDispatcher started: max_wheel_speed=%.3f",
    max_wheel_speed_);
}

void WheelSpeedsDispatcher::declare_parameters()
{
  this->declare_parameter("max_wheel_speed", 1.0);
  this->declare_parameter("front_left_topic", std::string("/front_left/desired_velocity"));
  this->declare_parameter("front_right_topic", std::string("/front_right/desired_velocity"));
  this->declare_parameter("rear_left_topic", std::string("/rear_left/desired_velocity"));
  this->declare_parameter("rear_right_topic", std::string("/rear_right/desired_velocity"));
}

void WheelSpeedsDispatcher::wheel_speeds_callback(
  const robot_interfaces::msg::WheelSpeeds::SharedPtr msg)
{
  const double inv = (max_wheel_speed_ > 0.0) ? (1.0 / max_wheel_speed_) : 0.0;

  auto publish_scaled = [inv](const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & pub, double value) {
    auto out = std::make_unique<std_msgs::msg::Float64>();
    out->data = std::clamp(value * inv, -1.0, 1.0);
    pub->publish(std::move(out));
  };

  publish_scaled(pub_fl_, msg->front_left_velocity);
  publish_scaled(pub_fr_, msg->front_right_velocity);
  publish_scaled(pub_rl_, msg->rear_left_velocity);
  publish_scaled(pub_rr_, msg->rear_right_velocity);
}

rcl_interfaces::msg::SetParametersResult WheelSpeedsDispatcher::on_parameter_event(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "ok";

  bool recreate_publishers = false;

  for (const auto & param : params) {
    const auto & name = param.get_name();
    if (name == "max_wheel_speed") {
      const double speed = param.as_double();
      if (speed <= 0.0) {
        result.successful = false;
        result.reason = "max_wheel_speed must be > 0";
        return result;
      }
      max_wheel_speed_ = speed;
    } else if (name == "front_left_topic") {
      front_left_topic_ = param.as_string();
      recreate_publishers = true;
    } else if (name == "front_right_topic") {
      front_right_topic_ = param.as_string();
      recreate_publishers = true;
    } else if (name == "rear_left_topic") {
      rear_left_topic_ = param.as_string();
      recreate_publishers = true;
    } else if (name == "rear_right_topic") {
      rear_right_topic_ = param.as_string();
      recreate_publishers = true;
    }
  }

  if (recreate_publishers) {
    pub_fl_ = this->create_publisher<std_msgs::msg::Float64>(front_left_topic_, rclcpp::SystemDefaultsQoS());
    pub_fr_ = this->create_publisher<std_msgs::msg::Float64>(front_right_topic_, rclcpp::SystemDefaultsQoS());
    pub_rl_ = this->create_publisher<std_msgs::msg::Float64>(rear_left_topic_, rclcpp::SystemDefaultsQoS());
    pub_rr_ = this->create_publisher<std_msgs::msg::Float64>(rear_right_topic_, rclcpp::SystemDefaultsQoS());
  }

  RCLCPP_INFO(this->get_logger(), "Updated max_wheel_speed=%.3f", max_wheel_speed_);
  return result;
}

}  // namespace mecanum_kinematics

RCLCPP_COMPONENTS_REGISTER_NODE(mecanum_kinematics::WheelSpeedsDispatcher)
