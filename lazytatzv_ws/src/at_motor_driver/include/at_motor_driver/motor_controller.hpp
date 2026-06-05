#ifndef AT_MOTOR_DRIVER__MOTOR_CONTROLLER_HPP_
#define AT_MOTOR_DRIVER__MOTOR_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/float64.hpp"

namespace at_motor_driver {

class MotorController : public rclcpp::Node {
 public:
  explicit MotorController(const rclcpp::NodeOptions & options);

 private:
  void desired_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void measured_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void control_tick();
  rcl_interfaces::msg::SetParametersResult on_parameter_event(
    const std::vector<rclcpp::Parameter>& params);

  // params
  bool use_outer_pid_;
  double kp_, ki_, kd_;
  double max_output_;
  double rate_hz_;
  std::string desired_topic_;
  std::string measured_topic_;
  std::string output_topic_;

  // state
  std::atomic<double> desired_{0.0};
  std::atomic<double> measured_{0.0};
  std::atomic<bool> have_measured_{false};

  double integrator_ = 0.0;
  double last_error_ = 0.0;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_desired_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_measured_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_target_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace at_motor_driver

#endif  // AT_MOTOR_DRIVER__MOTOR_CONTROLLER_HPP_
