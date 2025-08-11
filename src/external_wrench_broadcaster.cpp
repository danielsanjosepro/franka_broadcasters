#include <franka_broadcasters/default_robot_behavior_utils.hpp>
#include <franka_broadcasters/external_wrench_broadcaster.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <binders.h>
#include <cassert>
#include <cmath>
#include <memory>
#include <rclcpp/logging.hpp>

using namespace std::chrono_literals;

namespace franka_broadcasters {

controller_interface::InterfaceConfiguration
ExternalWrenchBroadcaster::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration
ExternalWrenchBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names =
      franka_robot_state_->get_state_interface_names();
  return state_interfaces_config;
}

controller_interface::return_type
ExternalWrenchBroadcaster::update(const rclcpp::Time &time,
                                  const rclcpp::Duration & /*period*/) {

  if (!franka_robot_state_) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Franka robot state not initialized");
    return controller_interface::return_type::ERROR;
  }

  if (!external_wrench_publisher_) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "External wrench publisher not initialized");
    return controller_interface::return_type::ERROR;
  }

  if (!franka_robot_state_->get_values_as_message(franka_robot_state_msg_)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                        "Failed to get robot state values");
    return controller_interface::return_type::ERROR;
  }
  
  geometry_msgs::msg::WrenchStamped wrench_msg;
  wrench_msg.header.stamp = time;
  wrench_msg.header.frame_id = params_.frame_id;
  
  // Extract external wrench from Franka robot state
  // K_F_ext_hat_K contains the external wrench in base frame
  auto raw_wrench = franka_robot_state_msg_.k_f_ext_hat_k.wrench;
  
  // Apply exponential moving average filter
  wrench_msg.wrench = ema_filter_.filter(raw_wrench);
  
  external_wrench_publisher_->publish(wrench_msg);

  return controller_interface::return_type::OK;
}

CallbackReturn ExternalWrenchBroadcaster::on_init() {
  try {
    // Initialize parameters
    params_listener_ =
        std::make_shared<external_wrench_broadcaster::ParamListener>(get_node());
    params_listener_->refresh_dynamic_parameters();
    params_ = params_listener_->get_params();

    // Validate parameters
    if (params_.arm_id.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "arm_id parameter cannot be empty");
      return CallbackReturn::ERROR;
    }
    if (params_.frame_id.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "frame_id parameter cannot be empty");
      return CallbackReturn::ERROR;
    }

    std::string robot_description;
    if (!get_node()->get_parameter("robot_description", robot_description)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to get robot_description parameter");
      return CallbackReturn::ERROR;
    }
    if (robot_description.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "robot_description parameter is empty");
      return CallbackReturn::ERROR;
    }

    if (!franka_robot_state_) {
      franka_robot_state_ =
          std::make_unique<franka_semantic_components::FrankaRobotState>(
              franka_semantic_components::FrankaRobotState(
                  params_.arm_id + "/" + state_interface_name,
                  robot_description));
    }

    // Use sensor data QoS for wrench publishing (reliable, keep last 10)
    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable)
                              .durability(rclcpp::DurabilityPolicy::Volatile);
    external_wrench_publisher_ =
        get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
            "external_wrench", qos);

    // Initialize the EMA filter with the configured alpha parameter
    ema_filter_.set_alpha(params_.filter_alpha);

    RCLCPP_INFO(get_node()->get_logger(),
                "ExternalWrenchBroadcaster initialized for arm: %s, frame: %s, filter_alpha: %.3f",
                params_.arm_id.c_str(), params_.frame_id.c_str(), params_.filter_alpha);
    return CallbackReturn::SUCCESS;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Exception during initialization: %s", e.what());
    return CallbackReturn::ERROR;
  }
}

CallbackReturn ExternalWrenchBroadcaster::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // Refresh parameters in case they changed
  if (params_listener_) {
    params_listener_->refresh_dynamic_parameters();
    params_ = params_listener_->get_params();
    // Update filter alpha if it changed
    ema_filter_.set_alpha(params_.filter_alpha);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn ExternalWrenchBroadcaster::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!franka_robot_state_) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Cannot activate: franka_robot_state_ is null");
    return CallbackReturn::ERROR;
  }
  if (state_interfaces_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Cannot activate: no state interfaces available");
    return CallbackReturn::ERROR;
  }
  franka_robot_state_->assign_loaned_state_interfaces(state_interfaces_);
  // Reset the filter when activating to ensure clean startup
  ema_filter_.reset();
  RCLCPP_INFO(get_node()->get_logger(), "ExternalWrenchBroadcaster activated");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ExternalWrenchBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (franka_robot_state_) {
    franka_robot_state_->release_interfaces();
  }
  RCLCPP_INFO(get_node()->get_logger(), "ExternalWrenchBroadcaster deactivated");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ExternalWrenchBroadcaster::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  franka_robot_state_.reset();
  external_wrench_publisher_.reset();
  params_listener_.reset();
  RCLCPP_INFO(get_node()->get_logger(), "ExternalWrenchBroadcaster cleaned up");
  return CallbackReturn::SUCCESS;
}

} // namespace franka_broadcasters
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_broadcasters::ExternalWrenchBroadcaster,
                       controller_interface::ControllerInterface)
