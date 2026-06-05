#include "motor_controller/motor_controller.hpp"
#include <chrono>
#include <vector>
#include "rclcpp_components/register_node_macro.hpp"

namespace motor_controller {

MotorController::MotorController(const rclcpp::NodeOptions & options)
: Node("motor_controller", options) {
  this->declare_parameter("use_outer_pid", true);
  this->declare_parameter("kp", 0.5);
  this->declare_parameter("ki", 0.1);
  this->declare_parameter("kd", 0.0);
  this->declare_parameter("max_output", 1.0);
  this->declare_parameter("rate_hz", 20.0);
  this->declare_parameter("desired_topic", std::string("/front_left/desired_velocity"));
  this->declare_parameter("measured_topic", std::string("/front_left/measured_velocity"));
  this->declare_parameter("output_topic", std::string("/front_left/target_velocity"));

  use_outer_pid_ = this->get_parameter("use_outer_pid").as_bool();
  kp_ = this->get_parameter("kp").as_double();
  ki_ = this->get_parameter("ki").as_double();
  kd_ = this->get_parameter("kd").as_double();
  max_output_ = this->get_parameter("max_output").as_double();
  rate_hz_ = this->get_parameter("rate_hz").as_double();
  desired_topic_ = this->get_parameter("desired_topic").as_string();
  measured_topic_ = this->get_parameter("measured_topic").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();

  sub_desired_ = this->create_subscription<std_msgs::msg::Float64>(
    desired_topic_, 10, std::bind(&MotorController::desired_callback, this, std::placeholders::_1));

  sub_measured_ = this->create_subscription<std_msgs::msg::Float64>(
    measured_topic_, 50, std::bind(&MotorController::measured_callback, this, std::placeholders::_1));

  pub_target_ = this->create_publisher<std_msgs::msg::Float64>(output_topic_, 10);

  const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&MotorController::control_tick, this));

  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&MotorController::on_parameter_event, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "MotorController started: desired='%s' measured='%s' output='%s'",
    desired_topic_.c_str(), measured_topic_.c_str(), output_topic_.c_str());
}

void MotorController::desired_callback(const std_msgs::msg::Float64::SharedPtr msg) {
  desired_.store(msg->data);
}

void MotorController::measured_callback(const std_msgs::msg::Float64::SharedPtr msg) {
  measured_.store(msg->data);
  have_measured_.store(true);
}

void MotorController::control_tick() {
  double desired = desired_.load();
  bool have_meas = have_measured_.load();
  double measured = measured_.load();

  std_msgs::msg::Float64 out_msg;

  if (!use_outer_pid_ || !have_meas) {
    out_msg.data = desired;
    pub_target_->publish(out_msg);
    return;
  }

  double error = desired - measured;
  double p = kp_ * error;
  integrator_ += error / rate_hz_;
  double i = ki_ * integrator_;
  double d = kd_ * (error - last_error_) * rate_hz_;

  double u = p + i + d + desired;

  if (u > max_output_) {
    u = max_output_;
    if (i > 0.0) integrator_ -= error / rate_hz_; 
  } else if (u < -max_output_) {
    u = -max_output_;
    if (i < 0.0) integrator_ -= error / rate_hz_;
  }

  last_error_ = error;
  out_msg.data = u;
  pub_target_->publish(out_msg);
}

rcl_interfaces::msg::SetParametersResult MotorController::on_parameter_event(
  const std::vector<rclcpp::Parameter>& params)
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;
  result.reason = "ok";

  bool recreate_required = false;

  for (const auto & param : params) {
    const auto & name = param.get_name();
    if (name == "use_outer_pid") {
      use_outer_pid_ = param.as_bool();
    } else if (name == "kp") {
      kp_ = param.as_double();
    } else if (name == "ki") {
      ki_ = param.as_double();
    } else if (name == "kd") {
      kd_ = param.as_double();
    } else if (name == "max_output") {
      max_output_ = param.as_double();
    } else if (name == "rate_hz") {
      const double rate = param.as_double();
      if (rate <= 0.0) {
        result.successful = false;
        result.reason = "rate_hz must be > 0";
        return result;
      }
      rate_hz_ = rate;
      recreate_required = true;
    } else if (name == "desired_topic") {
      desired_topic_ = param.as_string();
      recreate_required = true;
    } else if (name == "measured_topic") {
      measured_topic_ = param.as_string();
      recreate_required = true;
    } else if (name == "output_topic") {
      output_topic_ = param.as_string();
      recreate_required = true;
    }
  }

  if (recreate_required) {
    sub_desired_.reset();
    sub_measured_.reset();
    pub_target_.reset();

    sub_desired_ = this->create_subscription<std_msgs::msg::Float64>(
      desired_topic_, 10, std::bind(&MotorController::desired_callback, this, std::placeholders::_1));
    sub_measured_ = this->create_subscription<std_msgs::msg::Float64>(
      measured_topic_, 50, std::bind(&MotorController::measured_callback, this, std::placeholders::_1));
    pub_target_ = this->create_publisher<std_msgs::msg::Float64>(output_topic_, 10);

    control_timer_->cancel();
    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MotorController::control_tick, this));
  }

  return result;
}

}  // namespace motor_controller

RCLCPP_COMPONENTS_REGISTER_NODE(motor_controller::MotorController)
