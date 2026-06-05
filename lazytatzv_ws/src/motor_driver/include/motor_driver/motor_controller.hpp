#ifndef MOTOR_DRIVER__MOTOR_CONTROLLER_HPP_
#define MOTOR_DRIVER__MOTOR_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace motor_driver {

class MotorController : public rclcpp::Node {
 public:
  explicit MotorController(const rclcpp::NodeOptions& options);

 private:
  void desired_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void measured_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void control_tick();
  rcl_interfaces::msg::SetParametersResult on_parameter_event(
    const std::vector<rclcpp::Parameter>& params);

  // PID Parameters
  bool use_outer_pid_{true};
  double kp_{0.5};
  double ki_{0.1};
  double kd_{0.0};
  double max_output_{1.0};
  double rate_hz_{20.0};

  // State
  double integrator_{0.0};
  double last_error_{0.0};
  std::atomic<double> desired_{0.0};
  std::atomic<double> measured_{0.0};
  std::atomic<bool> have_measured_{false};

  // Topics
  std::string desired_topic_;
  std::string measured_topic_;
  std::string output_topic_;

  // ROS
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_desired_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_measured_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_target_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace motor_driver

#endif  // MOTOR_DRIVER__MOTOR_CONTROLLER_HPP_
