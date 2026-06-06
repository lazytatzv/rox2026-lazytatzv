#include "robstride_driver/robstride_at_node.hpp"
#include <cmath>
#include <algorithm>
#include "rclcpp_components/register_node_macro.hpp"

namespace robstride_driver {

using namespace at_protocol;

RobstrideAtNode::RobstrideAtNode(const rclcpp::NodeOptions& options) 
: rclcpp_lifecycle::LifecycleNode("robstride_at_node", options) {
  this->declare_parameter("motor_id", 0x0C);
  this->declare_parameter("joint_name", "motor_joint");
  this->declare_parameter("invert_direction", false);
  this->declare_parameter("max_speed_limit_percentage", 50.0);
  this->declare_parameter("topic_tx_queue", "/communication/tx_queue");
  this->declare_parameter("topic_rx_queue", "/communication/rx_queue");
  this->declare_parameter("topic_velocity_command", "~/velocity_command");

  this->declare_parameter("position_min_rad", -12.57);
  this->declare_parameter("position_max_rad", 12.57);
  this->declare_parameter("velocity_min_rad_s", -50.0);
  this->declare_parameter("velocity_max_rad_s", 50.0);
  this->declare_parameter("torque_min_nm", -6.0);
  this->declare_parameter("torque_max_nm", 6.0);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobstrideAtNode::on_configure(const rclcpp_lifecycle::State &)
{
  motor_id_ = static_cast<uint8_t>(this->get_parameter("motor_id").as_int());
  joint_name_ = this->get_parameter("joint_name").as_string();
  invert_direction_ = this->get_parameter("invert_direction").as_bool();
  topic_tx_queue_ = this->get_parameter("topic_tx_queue").as_string();
  topic_rx_queue_ = this->get_parameter("topic_rx_queue").as_string();
  topic_velocity_command_ = this->get_parameter("topic_velocity_command").as_string();

  pos_min_ = this->get_parameter("position_min_rad").as_double();
  pos_max_ = this->get_parameter("position_max_rad").as_double();
  vel_min_ = this->get_parameter("velocity_min_rad_s").as_double();
  vel_max_ = this->get_parameter("velocity_max_rad_s").as_double();
  tor_min_ = this->get_parameter("torque_min_nm").as_double();
  tor_max_ = this->get_parameter("torque_max_nm").as_double();
  
  double max_speed_percentage = this->get_parameter("max_speed_limit_percentage").as_double();
  max_at_command_delta_ = static_cast<int>(NEUTRAL_VELOCITY_VALUE * (max_speed_percentage / 100.0));

  publisher_serial_frames_ = this->create_publisher<robot_interfaces::msg::SerialFrame>(topic_tx_queue_, 10);
  publisher_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
  
  subscription_velocity_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    topic_velocity_command_, 10, std::bind(&RobstrideAtNode::velocity_callback, this, std::placeholders::_1));

  subscription_serial_rx_ = this->create_subscription<robot_interfaces::msg::SerialFrame>(
    topic_rx_queue_, 50, std::bind(&RobstrideAtNode::serial_rx_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Configured motor 0x%02X", motor_id_);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobstrideAtNode::on_activate(const rclcpp_lifecycle::State &)
{
  publisher_serial_frames_->on_activate();
  publisher_joint_state_->on_activate();
  send_enable_command();
  RCLCPP_INFO(get_logger(), "Activated & Motor Enabled");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobstrideAtNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  send_disable_command();
  publisher_serial_frames_->on_deactivate();
  publisher_joint_state_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Deactivated & Motor Disabled");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobstrideAtNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  publisher_serial_frames_.reset();
  publisher_joint_state_.reset();
  subscription_velocity_.reset();
  subscription_serial_rx_.reset();
  RCLCPP_INFO(get_logger(), "Cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobstrideAtNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

double RobstrideAtNode::uint_to_float(uint16_t value, double low, double high) {
  double span = high - low;
  return static_cast<double>(value) * span / 65535.0 + low;
}

void RobstrideAtNode::send_enable_command() {
  auto frame = std::make_unique<robot_interfaces::msg::SerialFrame>();
  frame->frame_data = {
    FRAME_HEADER_A, FRAME_HEADER_T, CMD_BASIC_CONFIG,
    DEFAULT_SOURCE_ID_HI, DEFAULT_SOURCE_ID_LO, motor_id_,
    DATA_LEN_8_BYTES, 0x00, REG_ADDR_MOTOR_ENABLE,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    FRAME_FOOTER_CR, FRAME_FOOTER_LF
  };
  publisher_serial_frames_->publish(std::move(frame));
}

void RobstrideAtNode::send_disable_command() {
  auto frame = std::make_unique<robot_interfaces::msg::SerialFrame>();
  frame->frame_data = {
    FRAME_HEADER_A, FRAME_HEADER_T, CMD_BASIC_CONFIG,
    DEFAULT_SOURCE_ID_HI, DEFAULT_SOURCE_ID_LO, motor_id_,
    DATA_LEN_8_BYTES, 0x00, REG_ADDR_MOTOR_ENABLE,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    FRAME_FOOTER_CR, FRAME_FOOTER_LF
  };
  publisher_serial_frames_->publish(std::move(frame));
}

void RobstrideAtNode::velocity_callback(const std_msgs::msg::Float64MultiArray::SharedPtr message) {
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
  if (message->data.empty()) return;

  double velocity_rad_s = message->data[0];
  if (invert_direction_) velocity_rad_s = -velocity_rad_s;
  velocity_rad_s = std::clamp(velocity_rad_s, vel_min_, vel_max_);

  int delta = static_cast<int>(std::round((velocity_rad_s / vel_max_) * max_at_command_delta_));
  uint16_t at_value = NEUTRAL_VELOCITY_VALUE + delta;
  uint8_t direction_flag = (at_value == NEUTRAL_VELOCITY_VALUE) ? DIR_STOP : DIR_ROTATING;

  auto frame = std::make_unique<robot_interfaces::msg::SerialFrame>();
  frame->frame_data = {
    FRAME_HEADER_A, FRAME_HEADER_T, CMD_DATA_STREAMING,
    DEFAULT_SOURCE_ID_HI, DEFAULT_SOURCE_ID_LO, motor_id_,
    DATA_LEN_8_BYTES, SPEED_CMD_INDICATOR, REG_ADDR_VELOCITY_CTRL,
    0x00, 0x00, CTRL_MODE_VELOCITY, direction_flag,
    static_cast<uint8_t>((at_value >> 8) & 0xFF),
    static_cast<uint8_t>(at_value & 0xFF),
    FRAME_FOOTER_CR, FRAME_FOOTER_LF
  };
  publisher_serial_frames_->publish(std::move(frame));
}

void RobstrideAtNode::serial_rx_callback(const robot_interfaces::msg::SerialFrame::SharedPtr message) {
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
  const auto& data = message->frame_data;
  
  if (data.size() < 16) return;
  if (data[0] != FRAME_HEADER_A || data[1] != FRAME_HEADER_T) return;
  if (data[5] != motor_id_) return;
  if (data.size() < 15) return; 

  uint16_t pos_u = (data[7] << 8) | data[8];
  uint16_t vel_u = (data[9] << 8) | data[10];
  uint16_t tor_u = (data[11] << 8) | data[12];

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

RCLCPP_COMPONENTS_REGISTER_NODE(robstride_driver::RobstrideAtNode)
