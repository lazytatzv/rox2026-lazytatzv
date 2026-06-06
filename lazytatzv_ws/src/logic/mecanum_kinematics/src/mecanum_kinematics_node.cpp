#include "mecanum_kinematics/mecanum_kinematics_node.hpp"
#include "mecanum_kinematics/kinematics.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <functional>
#include <vector>
#include <cmath>

namespace mecanum_kinematics {

MecanumKinematicsNode::MecanumKinematicsNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("mecanum_kinematics_node", options)
{
  declare_parameters();
}

void MecanumKinematicsNode::declare_parameters()
{
  this->declare_parameter("half_length", 0.12);
  this->declare_parameter("half_width", 0.10);
  this->declare_parameter("wheel_radius", 0.05);
  this->declare_parameter("topic_cmd_vel", "cmd_vel");
  this->declare_parameter("topic_wheel_speeds", "wheel_speeds");
}

void MecanumKinematicsNode::update_parameters()
{
  half_length_ = this->get_parameter("half_length").as_double();
  half_width_ = this->get_parameter("half_width").as_double();
  wheel_radius_ = this->get_parameter("wheel_radius").as_double();
  topic_cmd_vel_ = this->get_parameter("topic_cmd_vel").as_string();
  topic_wheel_speeds_ = this->get_parameter("topic_wheel_speeds").as_string();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MecanumKinematicsNode::on_configure(const rclcpp_lifecycle::State &)
{
  update_parameters();
  
  publisher_wheel_speeds_ = this->create_publisher<robot_interfaces::msg::WheelSpeeds>(
    topic_wheel_speeds_, rclcpp::SystemDefaultsQoS());

  publisher_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SystemDefaultsQoS());

  subscription_command_velocity_ = this->create_subscription<geometry_msgs::msg::Twist>(
    topic_cmd_vel_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MecanumKinematicsNode::command_velocity_callback, this, std::placeholders::_1));

  subscription_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MecanumKinematicsNode::joint_state_callback, this, std::placeholders::_1));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(get_logger(), "Configured: cmd_vel='%s' wheel_speeds='%s'", 
    topic_cmd_vel_.c_str(), topic_wheel_speeds_.c_str());
    
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MecanumKinematicsNode::on_activate(const rclcpp_lifecycle::State &)
{
  publisher_wheel_speeds_->on_activate();
  publisher_odom_->on_activate();
  last_time_ = this->now();
  first_odom_ = true;
  RCLCPP_INFO(get_logger(), "Activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MecanumKinematicsNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  publisher_wheel_speeds_->on_deactivate();
  publisher_odom_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MecanumKinematicsNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  publisher_wheel_speeds_.reset();
  publisher_odom_.reset();
  subscription_command_velocity_.reset();
  subscription_joint_states_.reset();
  tf_broadcaster_.reset();
  RCLCPP_INFO(get_logger(), "Cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MecanumKinematicsNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void MecanumKinematicsNode::command_velocity_callback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!publisher_wheel_speeds_->is_activated()) return;

  const auto wheels = mecanum_kinematics::compute_wheel_speeds(
    msg->linear.x, msg->linear.y, msg->angular.z,
    half_length_, half_width_, wheel_radius_);

  auto out = std::make_unique<robot_interfaces::msg::WheelSpeeds>();
  out->front_left_velocity = wheels[0];
  out->front_right_velocity = wheels[1];
  out->rear_left_velocity = wheels[2];
  out->rear_right_velocity = wheels[3];

  publisher_wheel_speeds_->publish(std::move(out));
}

void MecanumKinematicsNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!publisher_odom_->is_activated()) return;

  // We need all 4 wheels to compute body twist
  // Mapping JointState to wheel order: FL, FR, RL, RR
  std::array<double, 4> wheel_speeds = {0.0, 0.0, 0.0, 0.0};
  int found_count = 0;

  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "front_left_wheel_joint") { wheel_speeds[0] = msg->velocity[i]; found_count++; }
    else if (msg->name[i] == "front_right_wheel_joint") { wheel_speeds[1] = msg->velocity[i]; found_count++; }
    else if (msg->name[i] == "rear_left_wheel_joint") { wheel_speeds[2] = msg->velocity[i]; found_count++; }
    else if (msg->name[i] == "rear_right_wheel_joint") { wheel_speeds[3] = msg->velocity[i]; found_count++; }
  }

  if (found_count < 4) return;

  // Forward Kinematics
  const auto twist = mecanum_kinematics::compute_body_twist(
    wheel_speeds, half_length_, half_width_, wheel_radius_);

  double vx = twist[0];
  double vy = twist[1];
  double omega = twist[2];

  // Integration (Simple Euler)
  rclcpp::Time now = msg->header.stamp;
  if (first_odom_) {
    last_time_ = now;
    first_odom_ = false;
    return;
  }
  double dt = (now - last_time_).seconds();
  last_time_ = now;

  double delta_x = (vx * cos(th_) - vy * sin(th_)) * dt;
  double delta_y = (vx * sin(th_) + vy * cos(th_)) * dt;
  double delta_th = omega * dt;

  x_ += delta_x;
  y_ += delta_y;
  th_ += delta_th;

  // Publish Odom
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, th_);
  odom.pose.pose.orientation = tf2::toMsg(q);

  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = omega;

  publisher_odom_->publish(odom);

  // Publish TF
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now;
  t.header.frame_id = "odom";
  t.child_frame_id = "base_footprint";
  t.transform.translation.x = x_;
  t.transform.translation.y = y_;
  t.transform.translation.z = 0.0;
  t.transform.rotation = tf2::toMsg(q);

  tf_broadcaster_->sendTransform(t);
}

}  // namespace mecanum_kinematics

RCLCPP_COMPONENTS_REGISTER_NODE(mecanum_kinematics::MecanumKinematicsNode)
