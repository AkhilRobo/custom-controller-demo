#include "controller_new/forward_velocity_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_robot_controller
{
controller_interface::CallbackReturn ForwardVelocityController::on_init()
{
  auto_declare<std::string>("joint", "");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForwardVelocityController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_name_ = get_node()->get_parameter("joint").as_string();
  if (joint_name_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joint' parameter not set.");
    return controller_interface::CallbackReturn::ERROR;
  }

  velocity_command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64>(
    "~/cmd_vel", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
      received_velocity_command_.set(msg->data);
    });
  
  RCLCPP_INFO(get_node()->get_logger(), "Configuration successful for joint: %s", joint_name_.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ForwardVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
  return config;
}

controller_interface::InterfaceConfiguration ForwardVelocityController::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn ForwardVelocityController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  received_velocity_command_.set(0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForwardVelocityController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ForwardVelocityController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  double new_velocity_command = 0.0;
  received_velocity_command_.get(new_velocity_command);

  command_interfaces_[0].set_value(new_velocity_command);

  return controller_interface::return_type::OK;
}

}  
PLUGINLIB_EXPORT_CLASS(
  my_robot_controller::ForwardVelocityController,
  controller_interface::ControllerInterface)