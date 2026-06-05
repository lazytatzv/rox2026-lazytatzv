#include "mecanum_kinematics/wheel_speeds_dispatcher.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <vector>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace mecanum_kinematics {

WheelSpeedsDispatcher::WheelSpeedsDispatcher(const rclcpp::NodeOptions & options)
: Node("wheel_speeds_dispatcher", options)
{
  declare_parameters();

  front_left_topic_ = this->get_parameter("front_left_topic").as_string();
  front_right_topic_ = this->get_parameter("front_right_topic").as_string();
  rear_left_topic_ = this->get_parameter("rear_left_topic").as_string();
  rear_right_topic_ = this->get_parameter("rear_right_topic").as_string();

  subscription_ = this->create_subscription<robot_interfaces::msg::WheelSpeeds>(
    "wheel_speeds",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&WheelSpeedsDispatcher::wheel_speeds_callback, this, std::placeholders::_1));

  pub_fl_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(front_left_topic_, rclcpp::SystemDefaultsQoS());
  pub_fr_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(front_right_topic_, rclcpp::SystemDefaultsQoS());
  pub_rl_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(rear_left_topic_, rclcpp::SystemDefaultsQoS());
  pub_rr_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(rear_right_topic_, rclcpp::SystemDefaultsQoS());

  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&WheelSpeedsDispatcher::on_parameter_event, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "WheelSpeedsDispatcher (Unified Interface) started");
}

void WheelSpeedsDispatcher::declare_parameters()
{
  this->declare_parameter("front_left_topic", std::string("/front_left/velocity_command"));
  this->declare_parameter("front_right_topic", std::string("/front_right/velocity_command"));
  this->declare_parameter("rear_left_topic", std::string("/rear_left/velocity_command"));
  this->declare_parameter("rear_right_topic", std::string("/rear_right/velocity_command"));
}

void WheelSpeedsDispatcher::wheel_speeds_callback(
  const robot_interfaces::msg::WheelSpeeds::SharedPtr msg)
{
  auto publish_vec = [](const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr & pub, double velocity) {
    auto out = std::make_unique<std_msgs::msg::Float64MultiArray>();
    out->data.push_back(velocity); // index 0: speed (rad/s)
    out->data.push_back(1.0);      // index 1: current limit (dummy)
    pub->publish(std::move(out));
  };

  publish_vec(pub_fl_, msg->front_left_velocity);
  publish_vec(pub_fr_, msg->front_right_velocity);
  publish_vec(pub_rl_, msg->rear_left_velocity);
  publish_vec(pub_rr_, msg->rear_right_velocity);
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
    if (name == "front_left_topic") {
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
    pub_fl_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(front_left_topic_, rclcpp::SystemDefaultsQoS());
    pub_fr_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(front_right_topic_, rclcpp::SystemDefaultsQoS());
    pub_rl_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(rear_left_topic_, rclcpp::SystemDefaultsQoS());
    pub_rr_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(rear_right_topic_, rclcpp::SystemDefaultsQoS());
  }

  return result;
}

}  // namespace mecanum_kinematics

RCLCPP_COMPONENTS_REGISTER_NODE(mecanum_kinematics::WheelSpeedsDispatcher)
