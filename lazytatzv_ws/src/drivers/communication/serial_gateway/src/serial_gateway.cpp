#include "serial_gateway/serial_gateway.hpp"
#include <boost/asio.hpp>
#include "rclcpp_components/register_node_macro.hpp"

namespace serial_gateway {

SerialGateway::SerialGateway(const rclcpp::NodeOptions& options) 
: rclcpp_lifecycle::LifecycleNode("serial_gateway", options) {
  this->declare_parameter("serial_port", "/dev/ttyUSB1");
  this->declare_parameter("baud_rate", 921600);
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

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SerialGateway::on_configure(const rclcpp_lifecycle::State &)
{
  init_serial_port();

  publisher_rx_frames_ = this->create_publisher<robot_interfaces::msg::SerialFrame>(
    "/communication/rx_queue", 100);

  subscription_serial_frames_ = this->create_subscription<robot_interfaces::msg::SerialFrame>(
    "/communication/tx_queue", 100, std::bind(&SerialGateway::serial_frame_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Configured on %s", this->get_parameter("serial_port").as_string().c_str());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SerialGateway::on_activate(const rclcpp_lifecycle::State &)
{
  publisher_rx_frames_->on_activate();
  
  // Start async read loop
  start_async_read();

  // Run io_context in a dedicated thread
  if (!io_thread_.joinable()) {
    io_thread_ = std::thread([this]() {
      try {
        if (io_context_->stopped()) io_context_->restart();
        io_context_->run();
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "IO context error: %s", e.what());
      }
    });
  }
    
  RCLCPP_INFO(get_logger(), "Activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SerialGateway::on_deactivate(const rclcpp_lifecycle::State &)
{
  publisher_rx_frames_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SerialGateway::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (io_context_) io_context_->stop();
  if (io_thread_.joinable()) io_thread_.join();
  if (serial_port_ && serial_port_->is_open()) serial_port_->close();

  subscription_serial_frames_.reset();
  publisher_rx_frames_.reset();
  
  RCLCPP_INFO(get_logger(), "Cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SerialGateway::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
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

  boost::asio::async_read_until(*serial_port_, read_buffer_, "\r\n",
    [this](const boost::system::error_code& ec, std::size_t bytes_transferred) {
      if (!ec) {
        if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          auto msg = std::make_unique<robot_interfaces::msg::SerialFrame>();
          std::istream is(&read_buffer_);
          msg->frame_data.resize(bytes_transferred);
          is.read(reinterpret_cast<char*>(msg->frame_data.data()), bytes_transferred);
          publisher_rx_frames_->publish(std::move(msg));
        }
        start_async_read();
      } else if (ec != boost::asio::error::operation_aborted) {
        RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", ec.message().c_str());
      }
    });
}

void SerialGateway::serial_frame_callback(const robot_interfaces::msg::SerialFrame::SharedPtr message) {
  if (!serial_port_ || !serial_port_->is_open()) return;
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
  
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
