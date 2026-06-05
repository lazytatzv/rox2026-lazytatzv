#include "serial_gateway/serial_gateway.hpp"
#include <boost/asio.hpp>
#include "rclcpp_components/register_node_macro.hpp"

namespace serial_gateway {

SerialGateway::SerialGateway(const rclcpp::NodeOptions& options) 
: Node("serial_gateway", options) {
  this->declare_parameter("serial_port", "/dev/ttyUSB1");
  this->declare_parameter("baud_rate", 921600);

  init_serial_port();

  publisher_rx_frames_ = this->create_publisher<robot_interfaces::msg::SerialFrame>(
    "/serial_bus/rx_queue", 100);

  subscription_serial_frames_ = this->create_subscription<robot_interfaces::msg::SerialFrame>(
    "/serial_bus/tx_queue", 100, std::bind(&SerialGateway::serial_frame_callback, this, std::placeholders::_1));
  
  // Start async read loop
  start_async_read();

  // Run io_context in a dedicated thread
  io_thread_ = std::thread([this]() {
    try {
      io_context_->run();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "IO context error: %s", e.what());
    }
  });
    
  RCLCPP_INFO(this->get_logger(), "Serial Gateway (Boost.Asio) started on %s at %ld baud", 
    this->get_parameter("serial_port").as_string().c_str(), 
    this->get_parameter("baud_rate").as_int());
}

SerialGateway::~SerialGateway() {
  if (io_context_) {
    io_context_->stop();
  }
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
  if (serial_port_ && serial_port_->is_open()) {
    serial_port_->close();
  }
}

void SerialGateway::init_serial_port() {
  std::string port_path = this->get_parameter("serial_port").as_string();
  int baud_rate = this->get_parameter("baud_rate").as_int();

  try {
    io_context_ = std::make_unique<boost::asio::io_context>();
    serial_port_ = std::make_unique<boost::asio::serial_port>(*io_context_, port_path);

    serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s. Error: %s", port_path.c_str(), e.what());
    throw;
  }
}

void SerialGateway::start_async_read() {
  if (!serial_port_ || !serial_port_->is_open()) return;

  // Read until AT protocol delimiter \r\n
  boost::asio::async_read_until(*serial_port_, read_buffer_, "\r\n",
    [this](const boost::system::error_code& ec, std::size_t bytes_transferred) {
      if (!ec) {
        auto msg = std::make_unique<robot_interfaces::msg::SerialFrame>();
        
        // Extract data from streambuf
        std::istream is(&read_buffer_);
        msg->frame_data.resize(bytes_transferred);
        is.read(reinterpret_cast<char*>(msg->frame_data.data()), bytes_transferred);
        
        publisher_rx_frames_->publish(std::move(msg));
        
        // Continue reading
        start_async_read();
      } else if (ec != boost::asio::error::operation_aborted) {
        RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", ec.message().c_str());
      }
    });
}

void SerialGateway::serial_frame_callback(const robot_interfaces::msg::SerialFrame::SharedPtr message) {
  if (!serial_port_ || !serial_port_->is_open()) return;
  
  // Use post to ensure write happens on the io_thread
  io_context_->post([this, message]() {
    try {
      boost::asio::write(*serial_port_, boost::asio::buffer(message->frame_data));
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        "Serial write failed: %s", e.what());
    }
  });
}

}  // namespace serial_gateway

RCLCPP_COMPONENTS_REGISTER_NODE(serial_gateway::SerialGateway)
