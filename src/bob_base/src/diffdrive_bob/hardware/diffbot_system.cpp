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

#include "diffdrive_bob/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "Wheel.hpp"

#define WHEEL_RADIUS 0.084

// Initialize pulse counters
int left_wheel_pulse_count = 0;
int right_wheel_pulse_count = 0;

double left_input_sig = 0;
double right_input_sig = 0;

// Initialize wheel directions
std::string left_wheel_direction = "IDLE";
std::string right_wheel_direction = "IDLE";

#define LEFTWHEEL_PULSES_PER_REV 98
#define RIGHTWHEEL_PULSES_PER_REV 98

bool isRunning = false;
void signalHandler(int signal)
{
    (void)signal;
    std::cout << "Received signal. Shutting down." << std::endl;
    isRunning = false;
}

void read_encoder_values(int *left_encoder_value, int *right_encoder_value)
{
    *left_encoder_value = left_wheel_pulse_count;
    *right_encoder_value = right_wheel_pulse_count;
}

void set_motor_direction(std::string left_motor_dir, std::string right_motor_dir)
{
    left_wheel_direction  = left_motor_dir;
    right_wheel_direction  = right_motor_dir;
}

void read_motor_signal(double linsig, double rinsig)
{
    left_input_sig  = linsig;
    right_input_sig  = rinsig;
}


// Left wheel callback function
void left_wheel_pulse(int tick)
{   
    if(left_wheel_direction == "FORWARD" && fabs(left_input_sig)>=MIN_SPEED)
        left_wheel_pulse_count+=tick;
    else if(left_wheel_direction== "BACKWARD" && fabs(left_input_sig)>=MIN_SPEED)
        left_wheel_pulse_count-=tick;
}

// Right wheel callback function
void right_wheel_pulse(int tick)
{
    if(right_wheel_direction == "FORWARD" && fabs(right_input_sig)>=MIN_SPEED)
        right_wheel_pulse_count+=tick;
    else if(right_wheel_direction == "BACKWARD" && fabs(right_input_sig)>=MIN_SPEED)
        right_wheel_pulse_count-=tick;
}

int gpioResult = pigpio_start(NULL,NULL);

MD::Motor leftMotor(gpioResult,LEFT_DIRECTION_PIN, LEFT_PWM_PIN, 100,CCW);
ENC::Encoder leftEncoder(gpioResult,LEFT_ENCODER_PIN,left_wheel_pulse, LEFTWHEEL_PULSES_PER_REV);
ALM::Alarm leftAlarm(gpioResult,LEFT_ALARM_PIN);

WH::Wheel left_wheel("left_wheel",LEFTWHEEL_PULSES_PER_REV,WHEEL_RADIUS,leftMotor, leftEncoder, leftAlarm);

MD::Motor rightMotor(gpioResult,RIGHT_DIRECTION_PIN, RIGHT_PWM_PIN,100,CW);
ENC::Encoder rightEncoder(gpioResult,RIGHT_ENCODER_PIN,right_wheel_pulse, RIGHTWHEEL_PULSES_PER_REV);
ALM::Alarm rightAlarm(gpioResult,RIGHT_ALARM_PIN);

WH::Wheel right_wheel("right_wheel",RIGHTWHEEL_PULSES_PER_REV,WHEEL_RADIUS,rightMotor, rightEncoder, rightAlarm);

namespace diffdrive_bob
{
hardware_interface::CallbackReturn DiffDriveBobHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveBobHardware"), "Initializing...");

  config_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  config_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  config_.enc_ticks_per_rev = std::stoi(info_.hardware_parameters["enc_ticks_per_rev"]);
  config_.loop_rate = std::stod(info_.hardware_parameters["loop_rate"]);

  leftWheel.setup(config_.left_wheel_name, config_.enc_ticks_per_rev);
  rightWheel.setup(config_.right_wheel_name, config_.enc_ticks_per_rev);

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveBobHardware"), "Finished initialization");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveBobHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    leftWheel.name, hardware_interface::HW_IF_POSITION, &leftWheel.position));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    leftWheel.name, hardware_interface::HW_IF_VELOCITY, &leftWheel.velocity));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    rightWheel.name, hardware_interface::HW_IF_POSITION, &rightWheel.position));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    rightWheel.name, hardware_interface::HW_IF_VELOCITY, &rightWheel.velocity));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveBobHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(leftWheel.name, hardware_interface::HW_IF_VELOCITY, &leftWheel.command));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(rightWheel.name, hardware_interface::HW_IF_VELOCITY, &rightWheel.command));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveBobHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveBobHardware"), "Configuring motors and encoders...");

  // int gpioResult = 0;

  // // Disable built-in pigpio signal handling
  // // Must be called before gpioInitialise()
  // int cfg = gpioCfgGetInternals();
  // cfg |= PI_CFG_NOSIGHANDLER;
  // gpioCfgSetInternals(cfg);

  // signal(SIGINT, signalHandler);

  // // Initialize the pigpio library
  // gpioResult = gpioInitialise();

  // if (gpioResult == PI_INIT_FAILED)
  // {
  //     RCLCPP_FATAL(
  //       rclcpp::get_logger("DiffDriveBobHardware"),
  //       "Error value = '%i'", gpioResult);
  //     return hardware_interface::CallbackReturn::ERROR;   
  // }

  // signal(SIGINT, signalHandler);                          

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveBobHardware"), "Successfully configured motors and encoders!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveBobHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  isRunning = true;

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveBobHardware"), "Starting controller ...please wait...");

  left_wheel.set_PID(75.0,175.0,3.0); 
  right_wheel.set_PID(75.0,175.0,3.0); 

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveBobHardware"), "Successfully Started!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveBobHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveBobHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveBobHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveBobHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    double delta_seconds = period.seconds();

    set_motor_direction(left_wheel.Motor.getDirection(),right_wheel.Motor.getDirection());
    read_encoder_values(&left_wheel.encoder_ticks, &right_wheel.encoder_ticks);
    read_motor_signal(left_wheel.u,right_wheel.u);

    right_wheel.update();
    left_wheel.update();

    // Calculate wheel positions and velocities
    double previous_position = leftWheel.position;

    leftWheel.position = left_wheel.position;
    leftWheel.velocity = (leftWheel.position - previous_position) / delta_seconds;

    previous_position = rightWheel.position;
    rightWheel.position = right_wheel.position;
    rightWheel.velocity = (rightWheel.position - previous_position) / delta_seconds;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type diffdrive_bob ::DiffDriveBobHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  double left_motor_counts_per_loop = leftWheel.command / leftWheel.rads_per_tick / config_.loop_rate;
  double right_motor_counts_per_loop = rightWheel.command / rightWheel.rads_per_tick / config_.loop_rate;

  left_wheel.set_speed(left_motor_counts_per_loop);
  right_wheel.set_speed(right_motor_counts_per_loop);

  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_bob

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_bob::DiffDriveBobHardware, hardware_interface::SystemInterface)