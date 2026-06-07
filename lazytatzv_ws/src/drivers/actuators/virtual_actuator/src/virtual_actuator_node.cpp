#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

namespace virtual_actuator {

class VirtualActuatorNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit VirtualActuatorNode(const rclcpp::NodeOptions& options)
      : rclcpp_lifecycle::LifecycleNode("virtual_motor", options) {
    this->declare_parameter("joint_name", "virtual_joint");
    this->declare_parameter("publish_rate_hz", 50.0);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override {
    joint_name_ = this->get_parameter("joint_name").as_string();
    double hz = this->get_parameter("publish_rate_hz").as_double();

    publisher_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
    
    subscription_velocity_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/velocity_command", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          if (!msg->data.empty()) target_velocity_ = msg->data[0];
        });

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
        std::bind(&VirtualActuatorNode::timer_callback, this));

    last_time_ = this->now();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override {
    publisher_joint_state_->on_activate();
    last_time_ = this->now();
    RCLCPP_INFO(get_logger(), "Virtual Motor [%s] Activated", joint_name_.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override {
    publisher_joint_state_->on_deactivate();
    target_velocity_ = 0.0;
    current_velocity_ = 0.0;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

 private:
  void timer_callback() {
    auto now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    // Simple 1st-order inertia simulation
    current_velocity_ += (target_velocity_ - current_velocity_) * 0.5;
    current_position_ += current_velocity_ * dt;

    if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      auto msg = std::make_unique<sensor_msgs::msg::JointState>();
      msg->header.stamp = now;
      msg->name.push_back(joint_name_);
      msg->position.push_back(current_position_);
      msg->velocity.push_back(current_velocity_);
      msg->effort.push_back(0.0);
      publisher_joint_state_->publish(std::move(msg));
    }
  }

  std::string joint_name_;
  double target_velocity_ = 0.0;
  double current_velocity_ = 0.0;
  double current_position_ = 0.0;
  
  rclcpp::Time last_time_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr publisher_joint_state_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_velocity_;
};

} // namespace virtual_actuator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(virtual_actuator::VirtualActuatorNode)
