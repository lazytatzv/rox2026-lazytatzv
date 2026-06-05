#include "robstride_driver/robstride_motor_node.hpp"
#include <cmath>
#include <algorithm>
#include "rclcpp_components/register_node_macro.hpp"

namespace robstride_driver {

using namespace at_protocol;

RobstrideMotorNode::RobstrideMotorNode(const rclcpp::NodeOptions& options) 
: Node("robstride_motor_node", options) {
  // Common Parameters
  this->declare_parameter("motor_id", 0x0C);
  this->declare_parameter("joint_name", "motor_joint");
  this->declare_parameter("invert_direction", false);
  this->declare_parameter("max_speed_limit_percentage", 50.0);
  this->declare_parameter("topic_tx_queue", "/serial_bus/tx_queue");
  this->declare_parameter("topic_rx_queue", "/serial_bus/rx_queue");
  this->declare_parameter("topic_velocity_command", "~/velocity_command");

  // Physics Scaling Parameters (EL05 Defaults)
  this->declare_parameter("position_min_rad", -12.57);
  this->declare_parameter("position_max_rad", 12.57);
  this->declare_parameter("velocity_min_rad_s", -50.0);
  this->declare_parameter("velocity_max_rad_s", 50.0);
  this->declare_parameter("torque_min_nm", -6.0);
  this->declare_parameter("torque_max_nm", 6.0);

  motor_id_ = static_cast<uint8_t>(this->get_parameter("motor_id").as_int());
  joint_name_ = this->get_parameter("joint_name").as_string();
  invert_direction_ = this->get_parameter("invert_direction").as_bool();
  topic_tx_queue_ = this->get_parameter("topic_tx_queue").as_string();
  topic_rx_queue_ = this->get_parameter("topic_rx_queue").as_string();
  topic_target_velocity_ = this->get_parameter("topic_velocity_command").as_string();

  pos_min_ = this->get_parameter("position_min_rad").as_double();
  pos_max_ = this->get_parameter("position_max_rad").as_double();
  vel_min_ = this->get_parameter("velocity_min_rad_s").as_double();
  vel_max_ = this->get_parameter("velocity_max_rad_s").as_double();
  tor_min_ = this->get_parameter("torque_min_nm").as_double();
  tor_max_ = this->get_parameter("torque_max_nm").as_double();
  
  double max_speed_percentage = this->get_parameter("max_speed_limit_percentage").as_double();
  max_at_command_delta_ = static_cast<int>(NEUTRAL_VELOCITY_VALUE * (max_speed_percentage / 100.0));

  // ROS Publishers & Subscriptions
  publisher_serial_frames_ = this->create_publisher<robot_interfaces::msg::SerialFrame>(topic_tx_queue_, 10);
  publisher_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
  
  subscription_velocity_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    topic_target_velocity_, 10, std::bind(&RobstrideMotorNode::velocity_callback, this, std::placeholders::_1));

  subscription_serial_rx_ = this->create_subscription<robot_interfaces::msg::SerialFrame>(
    topic_rx_queue_, 50, std::bind(&RobstrideMotorNode::serial_rx_callback, this, std::placeholders::_1));

  // Sending enable command on startup
  enable_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), [this]() {
      this->send_enable_command();
      this->enable_timer_->cancel();
    });

  RCLCPP_INFO(this->get_logger(), "Robstride Motor Node (ID: 0x%02X, Joint: %s) started", motor_id_, joint_name_.c_str());
}

double RobstrideMotorNode::uint_to_float(uint16_t value, double low, double high) {
  double span = high - low;
  return static_cast<double>(value) * span / 65535.0 + low;
}

void RobstrideMotorNode::send_enable_command() {
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

void RobstrideMotorNode::velocity_callback(const std_msgs::msg::Float64MultiArray::SharedPtr message) {
  if (message->data.empty()) return;

  double velocity_rad_s = message->data[0];
  if (invert_direction_) velocity_rad_s = -velocity_rad_s;

  // Clamp to physical limits
  velocity_rad_s = std::clamp(velocity_rad_s, vel_min_, vel_max_);

  // Map rad/s to AT command value
  // Mapping: vel_max_ (50.0) -> NEUTRAL + max_at_command_delta_
  //          0.0              -> NEUTRAL
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

void RobstrideMotorNode::serial_rx_callback(const robot_interfaces::msg::SerialFrame::SharedPtr message) {
  const auto& data = message->frame_data;
  
  // AT Protocol Header Check (AT ...)
  if (data.size() < 16) return; // Basic AT frame size
  if (data[0] != FRAME_HEADER_A || data[1] != FRAME_HEADER_T) return;

  // Check if this frame is from our motor
  uint8_t sender_id = data[5];
  if (sender_id != motor_id_) return;

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

RCLCPP_COMPONENTS_REGISTER_NODE(robstride_driver::RobstrideMotorNode)
