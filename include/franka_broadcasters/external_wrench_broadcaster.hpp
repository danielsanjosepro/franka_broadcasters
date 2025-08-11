#pragma once
#include <Eigen/Dense>

#include <controller_interface/controller_interface.hpp>
#include <franka_broadcasters/external_wrench_broadcaster_parameters.hpp>
#include <franka_msgs/msg/franka_robot_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "franka_semantic_components/franka_robot_state.hpp"

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_broadcasters {

class ExponentialMovingAverageFilter {
public:
  ExponentialMovingAverageFilter(double alpha = 0.1) : alpha_(alpha), initialized_(false) {}
  
  geometry_msgs::msg::Wrench filter(const geometry_msgs::msg::Wrench& input) {
    if (!initialized_) {
      filtered_wrench_ = input;
      initialized_ = true;
      return filtered_wrench_;
    }
    
    filtered_wrench_.force.x = alpha_ * input.force.x + (1.0 - alpha_) * filtered_wrench_.force.x;
    filtered_wrench_.force.y = alpha_ * input.force.y + (1.0 - alpha_) * filtered_wrench_.force.y;
    filtered_wrench_.force.z = alpha_ * input.force.z + (1.0 - alpha_) * filtered_wrench_.force.z;
    filtered_wrench_.torque.x = alpha_ * input.torque.x + (1.0 - alpha_) * filtered_wrench_.torque.x;
    filtered_wrench_.torque.y = alpha_ * input.torque.y + (1.0 - alpha_) * filtered_wrench_.torque.y;
    filtered_wrench_.torque.z = alpha_ * input.torque.z + (1.0 - alpha_) * filtered_wrench_.torque.z;
    
    return filtered_wrench_;
  }
  
  void reset() {
    initialized_ = false;
  }
  
  void set_alpha(double alpha) {
    alpha_ = alpha;
  }

private:
  double alpha_;
  bool initialized_;
  geometry_msgs::msg::Wrench filtered_wrench_;
};

class ExternalWrenchBroadcaster
    : public controller_interface::ControllerInterface {
public:
  [[nodiscard]] controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;
  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  CallbackReturn on_init() override;
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

private:
  std::shared_ptr<external_wrench_broadcaster::ParamListener> params_listener_;
  external_wrench_broadcaster::Params params_;

  std::string state_interface_name{"robot_state"};

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>
      external_wrench_publisher_;

  franka_msgs::msg::FrankaRobotState franka_robot_state_msg_;
  std::unique_ptr<franka_semantic_components::FrankaRobotState>
      franka_robot_state_;
  
  ExponentialMovingAverageFilter ema_filter_;
};

} // namespace franka_broadcasters
