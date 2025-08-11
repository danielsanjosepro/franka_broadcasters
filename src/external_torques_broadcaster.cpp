#include <franka_broadcasters/default_robot_behavior_utils.hpp>
#include <franka_broadcasters/external_torques_broadcaster.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <binders.h>
#include <cassert>
#include <cmath>
#include <memory>
#include <rclcpp/logging.hpp>

using namespace std::chrono_literals;

namespace franka_broadcasters {

controller_interface::InterfaceConfiguration
ExternalTorquesBroadcaster::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration
ExternalTorquesBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names =
      franka_robot_state_->get_state_interface_names();
  return state_interfaces_config;
}

controller_interface::return_type
ExternalTorquesBroadcaster::update(const rclcpp::Time &time,
                                   const rclcpp::Duration & /*period*/) {

  if (!franka_robot_state_->get_values_as_message(franka_robot_state_msg_)) {
    return controller_interface::return_type::ERROR;
  }
  franka_robot_state_msg_.tau_ext_hat_filtered.name = params_.joints;
  franka_robot_state_msg_.tau_ext_hat_filtered.header.stamp = time;
  franka_robot_state_msg_.tau_ext_hat_filtered.header.frame_id = "world";
  external_joint_torques_publisher_->publish(
      franka_robot_state_msg_.tau_ext_hat_filtered);

  return controller_interface::return_type::OK;
}

CallbackReturn ExternalTorquesBroadcaster::on_init() {
  // Initialize parameters
  params_listener_ =
      std::make_shared<external_torques_broadcaster::ParamListener>(get_node());
  params_listener_->refresh_dynamic_parameters();
  params_ = params_listener_->get_params();

  std::string robot_description;
  if (!get_node()->get_parameter("robot_description", robot_description)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to get robot_description parameter");
    return CallbackReturn::ERROR;
  }
  if (!franka_robot_state_) {
    franka_robot_state_ =
        std::make_unique<franka_semantic_components::FrankaRobotState>(
            franka_semantic_components::FrankaRobotState(
                params_.arm_id + "/" + state_interface_name,
                robot_description));
  }

  external_joint_torques_publisher_ =
      get_node()->create_publisher<sensor_msgs::msg::JointState>(
          "external_torques_joint_state", rclcpp::SystemDefaultsQoS());
  return CallbackReturn::SUCCESS;
}

CallbackReturn ExternalTorquesBroadcaster::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  return CallbackReturn::SUCCESS;
}

CallbackReturn ExternalTorquesBroadcaster::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  franka_robot_state_->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ExternalTorquesBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  franka_robot_state_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

} // namespace franka_broadcasters
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_broadcasters::ExternalTorquesBroadcaster,
                       controller_interface::ControllerInterface)
