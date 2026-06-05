#include "motor_driver/ddsm_motor_node.hpp"
#include <cmath>
#include <algorithm>
#include "rclcpp_components/register_node_macro.hpp"

namespace motor_driver {

using namespace ddsm_protocol;

DdsmMotorNode::DdsmMotorNode(const rclcpp::NodeOptions& options) 
: Node("ddsm_motor_node", options) {
  this->declare_parameter("motor_id", 0x01);
  this->declare_parameter("invert_direction", false);
  this->declare_parameter("topic_tx_queue", "/serial_bus/tx_queue");
  this->declare_parameter("topic_target_velocity", "~/target_velocity");

  motor_id_ = static_cast<uint8_t>(this->get_parameter("motor_id").as_int());
  invert_direction_ = this->get_parameter("invert_direction").as_bool();
  topic_tx_queue_ = this->get_parameter("topic_tx_queue").as_string();
  topic_target_velocity_ = this->get_parameter("topic_target_velocity").as_string();
  
  publisher_serial_frames_ = this->create_publisher<robot_interfaces::msg::SerialFrame>(topic_tx_queue_, 10);
  
  subscription_velocity_ = this->create_subscription<std_msgs::msg::Float64>(
    topic_target_velocity_, 10, std::bind(&DdsmMotorNode::velocity_callback, this, std::placeholders::_1));

  // Switch to velocity loop mode on startup
  init_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), [this]() {
      this->switch_to_velocity_mode();
      this->init_timer_->cancel();
    });

  RCLCPP_INFO(this->get_logger(), "DDSM Motor Node (ID: 0x%02X) started", motor_id_);
}

void DdsmMotorNode::switch_to_velocity_mode() {
  auto frame = std::make_unique<robot_interfaces::msg::SerialFrame>();
  frame->frame_data = {motor_id_, MODE_SWITCH_CMD, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, MODE_VELOCITY};
  // DDSM mode switch command uses mode as last byte instead of CRC? 
  // Based on doc: 01 A0 00 00 00 00 00 00 00 02 (02 is mode)
  publisher_serial_frames_->publish(std::move(frame));
}

void DdsmMotorNode::velocity_callback(const std_msgs::msg::Float64::SharedPtr message) {
  double velocity_rpm = message->data; // Logic layer sends raw value, but DDSM expects RPM
  // Note: DDSM115 max is 330 rpm.
  if (invert_direction_) velocity_rpm = -velocity_rpm;

  int16_t rpm_val = static_cast<int16_t>(std::clamp(velocity_rpm, -330.0, 330.0));

  auto frame = std::make_unique<robot_interfaces::msg::SerialFrame>();
  frame->frame_data.resize(10);
  frame->frame_data[0] = motor_id_;
  frame->frame_data[1] = DRIVE_CMD;
  frame->frame_data[2] = static_cast<uint8_t>((rpm_val >> 8) & 0xFF);
  frame->frame_data[3] = static_cast<uint8_t>(rpm_val & 0xFF);
  frame->frame_data[4] = 0x00;
  frame->frame_data[5] = 0x00;
  frame->frame_data[6] = 0x00; // Acceleration time (0 = default)
  frame->frame_data[7] = 0x00; // Brake
  frame->frame_data[8] = 0x00;
  frame->frame_data[9] = calculate_crc8(frame->frame_data, 9);

  publisher_serial_frames_->publish(std::move(frame));
}

}  // namespace motor_driver

RCLCPP_COMPONENTS_REGISTER_NODE(motor_driver::DdsmMotorNode)
