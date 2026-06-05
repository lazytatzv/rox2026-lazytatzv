#ifndef SERIAL_GATEWAY__SERIAL_GATEWAY_HPP_
#define SERIAL_GATEWAY__SERIAL_GATEWAY_HPP_

#include <boost/asio.hpp>
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/serial_frame.hpp"

namespace serial_gateway {

class SerialGateway : public rclcpp::Node {
 public:
  explicit SerialGateway(const rclcpp::NodeOptions& options);
  virtual ~SerialGateway();

 private:
  void serial_frame_callback(const robot_interfaces::msg::SerialFrame::SharedPtr message);

  // Boost.Asio components
  std::unique_ptr<boost::asio::io_context> io_context_;
  std::unique_ptr<boost::asio::serial_port> serial_port_;

  rclcpp::Subscription<robot_interfaces::msg::SerialFrame>::SharedPtr subscription_serial_frames_;
};

}  // namespace serial_gateway

#endif  // SERIAL_GATEWAY__SERIAL_GATEWAY_HPP_
