#ifndef MY_ROBOT_CONTROLLER__FORWARD_VELOCITY_CONTROLLER_HPP_
#define MY_ROBOT_CONTROLLER__FORWARD_VELOCITY_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/float64.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_box.h"

namespace my_robot_controller
{
class ForwardVelocityController : public controller_interface::ControllerInterface
{
public:
  // Standard lifecycle methods for a controller
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Specify the command and state interfaces the controller needs
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // The main control loop function
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Name of the joint this controller controls
  std::string joint_name_;

  // ROS 2 subscriber for velocity commands
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_command_subscriber_;

  // Realtime-safe storage for the last received command
  realtime_tools::RealtimeBox<double> received_velocity_command_{0.0};
};
}  // namespace my_robot_controller

#endif  // MY_ROBOT_CONTROLLER__FORWARD_VELOCITY_CONTROLLER_HPP_