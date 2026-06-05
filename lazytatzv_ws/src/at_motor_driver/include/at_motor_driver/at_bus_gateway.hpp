#ifndef AT_MOTOR_DRIVER__AT_BUS_GATEWAY_HPP_
#define AT_MOTOR_DRIVER__AT_BUS_GATEWAY_HPP_

#include <boost/asio.hpp>
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/at_frame.hpp"

namespace at_motor_driver {

class AtBusGateway : public rclcpp::Node {
 public:
  explicit AtBusGateway(const rclcpp::NodeOptions& options);
  virtual ~AtBusGateway();

 private:
  void at_frame_callback(const robot_interfaces::msg::AtFrame::SharedPtr message);

  // Boost.Asio components
  std::unique_ptr<boost::asio::io_context> io_context_;
  std::unique_ptr<boost::asio::serial_port> serial_port_;

  rclcpp::Subscription<robot_interfaces::msg::AtFrame>::SharedPtr subscription_at_frames_;
};

}  // namespace at_motor_driver

#endif  // AT_MOTOR_DRIVER__AT_BUS_GATEWAY_HPP_
