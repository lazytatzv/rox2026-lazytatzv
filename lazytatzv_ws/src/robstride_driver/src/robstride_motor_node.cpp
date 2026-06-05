#include "robstride_driver/robstride_motor_node.hpp"
#include <cmath>
#include "rclcpp_components/register_node_macro.hpp"

namespace robstride_driver {

using namespace at_protocol;

RobstrideMotorNode::RobstrideMotorNode(const rclcpp::NodeOptions& options) 
: Node("robstride_motor_node", options) {
  this->declare_parameter("motor_id", 0x0C);
  this->declare_parameter("invert_direction", false);
  this->declare_parameter("max_speed_limit_percentage", 50.0);
  this->declare_parameter("topic_tx_queue", "/serial_bus/tx_queue");
  this->declare_parameter("topic_target_velocity", "~/target_velocity");

  motor_id_ = static_cast<uint8_t>(this->get_parameter("motor_id").as_int());
  invert_direction_ = this->get_parameter("invert_direction").as_bool();
  double max_speed_percentage = this->get_parameter("max_speed_limit_percentage").as_double();
  topic_tx_queue_ = this->get_parameter("topic_tx_queue").as_string();
  topic_target_velocity_ = this->get_parameter("topic_target_velocity").as_string();
  
  max_at_command_delta_ = static_cast<int>(NEUTRAL_VELOCITY_VALUE * (max_speed_percentage / 100.0));

  publisher_serial_frames_ = this->create_publisher<robot_interfaces::msg::SerialFrame>(topic_tx_queue_, 10);
  
  subscription_velocity_ = this->create_subscription<std_msgs::msg::Float64>(
    topic_target_velocity_, 10, std::bind(&RobstrideMotorNode::velocity_callback, this, std::placeholders::_1));

  // Sending enable command on startup
  enable_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), [this]() {
      this->send_enable_command();
      this->enable_timer_->cancel();
    });

  RCLCPP_INFO(this->get_logger(), "Robstride Motor Node (ID: 0x%02X) started", motor_id_);
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

void RobstrideMotorNode::velocity_callback(const std_msgs::msg::Float64::SharedPtr message) {
  double velocity = message->data;
  if (invert_direction_) velocity = -velocity;

  int delta = static_cast<int>(std::round(velocity * max_at_command_delta_));
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

}  // namespace robstride_driver

RCLCPP_COMPONENTS_REGISTER_NODE(robstride_driver::RobstrideMotorNode)
