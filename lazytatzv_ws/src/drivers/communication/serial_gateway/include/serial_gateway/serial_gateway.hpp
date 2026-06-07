// Copyright 2026 Tatsukiyano

#ifndef SERIAL_GATEWAY__SERIAL_GATEWAY_HPP_
#define SERIAL_GATEWAY__SERIAL_GATEWAY_HPP_

#include <boost/asio.hpp>
#include <thread>
#include <atomic>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "robot_interfaces/msg/serial_frame.hpp"

namespace serial_gateway {

class SerialGateway : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit SerialGateway(const rclcpp::NodeOptions & options);
  virtual ~SerialGateway();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

 private:
  void serial_frame_callback(const robot_interfaces::msg::SerialFrame::SharedPtr message);
  bool init_serial_port();
  void start_async_read();
  void try_reconnect();

  // Diagnostics
  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Boost.Asio components
  std::unique_ptr<boost::asio::io_context> io_context_;
  std::unique_ptr<boost::asio::serial_port> serial_port_;
  boost::asio::streambuf read_buffer_;
  std::thread io_thread_;

  // State & Recovery
  std::atomic<bool> is_connected_{false};
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  uint64_t tx_count_ = 0;
  uint64_t rx_count_ = 0;
  std::string last_error_ = "None";

  // Diagnostics
  std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;

  rclcpp::Subscription<robot_interfaces::msg::SerialFrame>::SharedPtr subscription_serial_frames_;
  rclcpp_lifecycle::LifecyclePublisher<robot_interfaces::msg::SerialFrame>::SharedPtr
  publisher_rx_frames_;
};

}  // namespace serial_gateway

#endif  // SERIAL_GATEWAY__SERIAL_GATEWAY_HPP_
