#include "at_motor_driver/at_bus_gateway.hpp"
#include <boost/asio.hpp>
#include "rclcpp_components/register_node_macro.hpp"

namespace at_motor_driver {

AtBusGateway::AtBusGateway(const rclcpp::NodeOptions& options) 
: Node("at_bus_gateway", options) {
  this->declare_parameter("serial_port", "/dev/ttyUSB1");
  this->declare_parameter("baud_rate", 921600);

  std::string port_path = this->get_parameter("serial_port").as_string();
  int baud_rate = this->get_parameter("baud_rate").as_int();

  try {
    io_context_ = std::make_unique<boost::asio::io_context>();
    serial_port_ = std::make_unique<boost::asio::serial_port>(*io_context_, port_path);

    // Configure serial port using Boost.Asio options
    serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s. Error: %s", port_path.c_str(), e.what());
    throw;
  }

  subscription_at_frames_ = this->create_subscription<robot_interfaces::msg::AtFrame>(
    "/at_bus/tx_queue", 100, std::bind(&AtBusGateway::at_frame_callback, this, std::placeholders::_1));
    
  RCLCPP_INFO(this->get_logger(), "AT Bus Gateway (Boost.Asio) started on %s at %d baud", 
    port_path.c_str(), baud_rate);
}

AtBusGateway::~AtBusGateway() {
  if (serial_port_ && serial_port_->is_open()) {
    serial_port_->close();
  }
}

void AtBusGateway::at_frame_callback(const robot_interfaces::msg::AtFrame::SharedPtr message) {
  if (!serial_port_ || !serial_port_->is_open()) return;
  
  try {
    // Synchronous write for simplicity in this version
    boost::asio::write(*serial_port_, boost::asio::buffer(message->frame_data));
  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
      "Serial write failed: %s", e.what());
  }
}

}  // namespace at_motor_driver

RCLCPP_COMPONENTS_REGISTER_NODE(at_motor_driver::AtBusGateway)
