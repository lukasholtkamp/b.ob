// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "bob_hardware.hpp"
#include <sstream>

int* pi_int;

namespace bob_base
{

DiffDriveBobHardware::DiffDriveBobHardware()
    : logger_(rclcpp::get_logger("DiffDriveBobHardware"))
{}

hardware_interface::CallbackReturn DiffDriveBobHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Initializing...");

  config_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  config_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  config_.enc_ticks_per_rev = std::stoi(info_.hardware_parameters["enc_ticks_per_rev"]);
  config_.loop_rate = std::stod(info_.hardware_parameters["loop_rate"]);

  // Set up wheels
  left_wheel_.setup(config_.left_wheel_name, config_.enc_ticks_per_rev);
  right_wheel_.setup(config_.right_wheel_name, config_.enc_ticks_per_rev);

  RCLCPP_INFO(logger_, "Finished initialization");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveBobHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.velocity));
  state_interfaces.emplace_back(hardware_interface::StateInterface(left_wheel_.name, hardware_interface::HW_IF_POSITION, &left_wheel_.position));
  state_interfaces.emplace_back(hardware_interface::StateInterface(right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.velocity));
  state_interfaces.emplace_back(hardware_interface::StateInterface(right_wheel_.name, hardware_interface::HW_IF_POSITION, &right_wheel_.position));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveBobHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.command));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.command));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveBobHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Configuring motors and encoders...");

  // Initialize pigpio using GPIO BCM pin numbers
  int pi = pigpio_start("172.22.2.165",NULL);
  pi_int = &pi;
  
  // Setup gpio outputs
  set_mode(pi,LEFT_DIRECTION_PIN,PI_OUTPUT);
  set_mode(pi,RIGHT_DIRECTION_PIN,PI_OUTPUT);
  set_mode(pi,LEFT_PWM_PIN,PI_OUTPUT);
  set_mode(pi,RIGHT_PWM_PIN,PI_OUTPUT);

  // Setup GPIO encoder interrupt and direction pins
  set_mode(pi,LEFT_ALARM_PIN,PI_INPUT);
  set_mode(pi,RIGHT_ALARM_PIN,PI_INPUT);
  set_mode(pi,LEFT_ENCODER_PIN,PI_INPUT);
  set_mode(pi,RIGHT_ENCODER_PIN,PI_INPUT);

  // Setup pull up resistors on encoder interrupt pins
  set_pull_up_down(pi,LEFT_ALARM_PIN,PI_PUD_DOWN);
  set_pull_up_down(pi,RIGHT_ALARM_PIN,PI_PUD_DOWN);
  set_pull_up_down(pi,LEFT_ENCODER_PIN,PI_PUD_UP);
  set_pull_up_down(pi,RIGHT_ENCODER_PIN,PI_PUD_UP);

  // Initialize encoder interrupts for falling signal states
  callback(pi,LEFT_ENCODER_PIN,RISING_EDGE,left_wheel_pulse);
  callback(pi,RIGHT_ENCODER_PIN,RISING_EDGE,right_wheel_pulse);

  gpio_write(pi,LEFT_DIRECTION_PIN,CCW);
  gpio_write(pi,RIGHT_DIRECTION_PIN,CW);                        

  RCLCPP_INFO(logger_, "Successfully configured motors and encoders!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveBobHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up ...please wait...");
  if (*pi_int>=0)
  {
    pigpio_stop(*pi_int);
  }
  RCLCPP_INFO(logger_, "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveBobHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(logger_, "Starting controller ...please wait...");
  RCLCPP_INFO(logger_, "Successfully Started!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveBobHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(logger_, "Deactivating ...please wait...");
  RCLCPP_INFO(logger_, "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveBobHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{

  // RCLCPP_INFO(logger_, "READ");

  // Obtain elapsed time
  double delta_seconds = period.seconds();

  // Obtain encoder values
  read_encoder_values(&left_wheel_.encoder_ticks, &right_wheel_.encoder_ticks);

  // Calculate wheel positions and velocities
  double previous_position = left_wheel_.position;
  left_wheel_.position = left_wheel_.calculate_encoder_angle();
  left_wheel_.velocity = (left_wheel_.position - previous_position) / delta_seconds;

  previous_position = right_wheel_.position;
  right_wheel_.position = right_wheel_.calculate_encoder_angle();
  right_wheel_.velocity = (right_wheel_.position - previous_position) / delta_seconds;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveBobHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  double left_motor_speed = ceil(351.1478 * left_wheel_.command);
  double right_motor_speed = ceil(351.1478 * right_wheel_.command);

  if(left_motor_speed>=255)
  {
    left_motor_speed=255;
  }
  else if(left_motor_speed<=-255)
  {
    left_motor_speed=-255;
  }

  if(right_motor_speed>=255)
  {
    right_motor_speed=255;
  }
  else if(right_motor_speed<=-255)
  {
    right_motor_speed=-255;
  }
  
  
  // Send commands to motor driver
  set_motor_speeds(*pi_int,left_motor_speed,right_motor_speed);

  // RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Left command %.5f", left_motor_speed);
  // RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Right command %.5f", right_motor_speed);

  return hardware_interface::return_type::OK;
}

}  // namespace bob_base

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  bob_base::DiffDriveBobHardware, hardware_interface::SystemInterface)
