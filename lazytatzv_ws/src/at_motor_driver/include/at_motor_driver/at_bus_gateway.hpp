#ifndef AT_MOTOR_DRIVER__AT_BUS_GATEWAY_HPP_
#define AT_MOTOR_DRIVER__AT_BUS_GATEWAY_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/at_frame.hpp"

namespace at_motor_driver {

class AtBusGateway : public rclcpp::Node {
 public:
  explicit AtBusGateway(const rclcpp::NodeOptions& options);
  ~AtBusGateway();

 private:
  void at_frame_callback(const robot_interfaces::msg::AtFrame::SharedPtr message);

  int serial_port_file_descriptor_{-1};
  rclcpp::Subscription<robot_interfaces::msg::AtFrame>::SharedPtr subscription_at_frames_;
};

}  // namespace at_motor_driver

#endif  // AT_MOTOR_DRIVER__AT_BUS_GATEWAY_HPP_
