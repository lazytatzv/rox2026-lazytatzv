#include "robstride_driver/robstride_can_node.hpp"
#include <cmath>
#include <algorithm>
#include "rclcpp_components/register_node_macro.hpp"

namespace robstride_driver {

RobstrideCanNode::RobstrideCanNode(const rclcpp::NodeOptions& options) 
: rclcpp_lifecycle::LifecycleNode("robstride_can_node", options) {
  this->declare_parameter("motor_id", 0x7F);
  this->declare_parameter("joint_name", "motor_joint");
  this->declare_parameter("invert_direction", false);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobstrideCanNode::on_configure(const rclcpp_lifecycle::State &)
{
  motor_id_ = this->get_parameter("motor_id").as_int();
  joint_name_ = this->get_parameter("joint_name").as_string();
  invert_direction_ = this->get_parameter("invert_direction").as_bool();

  publisher_can_tx_ = this->create_publisher<seeed_usb_can_analyzer_driver::msg::CanFrame>("/communication/tx_queue", 10);
  publisher_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
  
  subscription_velocity_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/velocity_command", 10, std::bind(&RobstrideCanNode::velocity_callback, this, std::placeholders::_1));

  subscription_can_rx_ = this->create_subscription<seeed_usb_can_analyzer_driver::msg::CanFrame>(
    "/communication/rx_queue", 50, std::bind(&RobstrideCanNode::can_rx_callback, this, std::placeholders::_1));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobstrideCanNode::on_activate(const rclcpp_lifecycle::State &)
{
  publisher_can_tx_->on_activate();
  publisher_joint_state_->on_activate();
  // Send Enable Command if needed
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobstrideCanNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  publisher_can_tx_->on_deactivate();
  publisher_joint_state_->on_deactivate();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobstrideCanNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  publisher_can_tx_.reset();
  publisher_joint_state_.reset();
  subscription_velocity_.reset();
  subscription_can_rx_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

double RobstrideCanNode::uint_to_float(uint16_t value, double low, double high) {
  double span = high - low;
  return static_cast<double>(value) * span / 65535.0 + low;
}

uint16_t RobstrideCanNode::float_to_uint(double value, double low, double high) {
  double span = high - low;
  if (value < low) value = low;
  else if (value > high) value = high;
  return static_cast<uint16_t>((value - low) * 65535.0 / span);
}

void RobstrideCanNode::velocity_callback(const std_msgs::msg::Float64MultiArray::SharedPtr message) {
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
  if (message->data.empty()) return;

  double velocity = message->data[0];
  if (invert_direction_) velocity = -velocity;

  // Robstride EL05 CAN Protocol for Velocity Control
  // Extended ID: (Mode << 24) | (Data << 8) | MotorID
  // Mode 2 (Velocity), Data = スケーリングされた速度
  uint16_t v_mapped = float_to_uint(velocity, vel_min_, vel_max_);
  
  seeed_usb_can_analyzer_driver::msg::CanFrame frame;
  frame.id = (2 << 24) | (v_mapped << 8) | (motor_id_ & 0xFF);
  frame.extended = true;
  frame.dlc = 0; // Control is often embedded in the ID for EL05
  
  publisher_can_tx_->publish(frame);
}

void RobstrideCanNode::can_rx_callback(const seeed_usb_can_analyzer_driver::msg::CanFrame::SharedPtr message) {
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
  
  // Parse Feedback from CAN ID/Data
  // Standard Robstride feedback usually has MotorID in the last 8 bits of ID
  if ((message->id & 0xFF) != motor_id_) return;

  // Extraction logic similar to Python version but for C++ CanFrame
  if (message->data.size() < 8) return;

  uint16_t pos_u = (message->data[1] << 8) | message->data[2];
  uint16_t vel_u = (message->data[3] << 8) | message->data[4];
  uint16_t tor_u = (message->data[5] << 8) | message->data[6];

  double position = uint_to_float(pos_u, pos_min_, pos_max_);
  double velocity = uint_to_float(vel_u, vel_min_, vel_max_);
  double torque = uint_to_float(tor_u, tor_min_, tor_max_);

  if (invert_direction_) {
    position = -position;
    velocity = -velocity;
    torque = -torque;
  }

  auto joint_state = std::make_unique<sensor_msgs::msg::JointState>();
  joint_state->header.stamp = this->now();
  joint_state->name.push_back(joint_name_);
  joint_state->position.push_back(position);
  joint_state->velocity.push_back(velocity);
  joint_state->effort.push_back(torque);

  publisher_joint_state_->publish(std::move(joint_state));
}

}  // namespace robstride_driver

RCLCPP_COMPONENTS_REGISTER_NODE(robstride_driver::RobstrideCanNode)
