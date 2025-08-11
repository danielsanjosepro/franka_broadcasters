#pragma once
#include <Eigen/Dense>

#include <controller_interface/controller_interface.hpp>
#include <franka_broadcasters/external_torques_broadcaster_parameters.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "franka_semantic_components/franka_robot_state.hpp"

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_broadcasters {

class ExternalTorquesBroadcaster
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

private:
  std::shared_ptr<external_torques_broadcaster::ParamListener> params_listener_;
  external_torques_broadcaster::Params params_;

  std::string state_interface_name{"robot_state"};

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>>
      external_joint_torques_publisher_;

  franka_msgs::msg::FrankaRobotState franka_robot_state_msg_;
  std::unique_ptr<franka_semantic_components::FrankaRobotState>
      franka_robot_state_;
};

} // namespace franka_broadcasters
