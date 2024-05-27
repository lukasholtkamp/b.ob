// Based on: https://github.com/TheNoobInventor/lidarbot/blob/main/lidarbot_base/src/lidarbot_hardware.cpp
// Date of Retrieval: 17.05.2024

#include "diffbot_system.hpp"
#include <sstream>

//! Same integer as pi_int but needed for the handler function to shut the motors down
int pi_sig;

namespace bob_base
{
  // Define logger
  DiffDriveBobHardware::DiffDriveBobHardware()
      : logger_(rclcpp::get_logger("DiffDriveBobHardware"))
  {
  }

  hardware_interface::CallbackReturn DiffDriveBobHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    // Check for errors
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger_, "Initializing...");

    // Read configuration parameters from the hardware information given in diffbot.ros2_control.xacro
    config_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    config_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    config_.enc_ticks_per_rev = std::stoi(info_.hardware_parameters["enc_ticks_per_rev"]);
    config_.loop_rate = std::stod(info_.hardware_parameters["loop_rate"]);

    // Set up wheels with names and the encoder ticks per revolution
    left_wheel_.setup(config_.left_wheel_name, config_.enc_ticks_per_rev);
    right_wheel_.setup(config_.right_wheel_name, config_.enc_ticks_per_rev);

    RCLCPP_INFO(logger_, "Finished initialization");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveBobHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(logger_, "Configuring motors and encoders...");

    // Initialize pigpio using GPIO pin numbers and save returned integer
    pi_int = pigpio_start(NULL, NULL);
    pi_sig = pi_int;

    // Setup gpio outputs

    // Motor Direction Pins
    set_mode(pi_int, LEFT_DIRECTION_PIN, PI_OUTPUT);
    set_mode(pi_int, RIGHT_DIRECTION_PIN, PI_OUTPUT);

    // Motor Speed Pins
    set_mode(pi_int, LEFT_PWM_PIN, PI_OUTPUT);
    set_mode(pi_int, RIGHT_PWM_PIN, PI_OUTPUT);

    // Setup Gpio Inputs

    // Direction Pins
    set_mode(pi_int, LEFT_ALARM_PIN, PI_INPUT);
    set_mode(pi_int, RIGHT_ALARM_PIN, PI_INPUT);

    // Encoder Pins
    set_mode(pi_int, LEFT_ENCODER_PIN, PI_INPUT);
    set_mode(pi_int, RIGHT_ENCODER_PIN, PI_INPUT);

    // Setup Pull down Resistors on Alarm Pins
    set_pull_up_down(pi_int, LEFT_ALARM_PIN, PI_PUD_DOWN);
    set_pull_up_down(pi_int, RIGHT_ALARM_PIN, PI_PUD_DOWN);

    // Setup Pull up Resistors on Encoder Pins
    set_pull_up_down(pi_int, LEFT_ENCODER_PIN, PI_PUD_UP);
    set_pull_up_down(pi_int, RIGHT_ENCODER_PIN, PI_PUD_UP);

    // Initialize Encoder interrupt for Rising Edge signal states with the functions defined in motor.cpp
    callback(pi_int, LEFT_ENCODER_PIN, RISING_EDGE, left_wheel_pulse);
    callback(pi_int, RIGHT_ENCODER_PIN, RISING_EDGE, right_wheel_pulse);

    // Set directions of motors to forward
    gpio_write(pi_int, LEFT_DIRECTION_PIN, CCW);
    gpio_write(pi_int, RIGHT_DIRECTION_PIN, CW);

    RCLCPP_INFO(logger_, "Successfully configured motors and encoders!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> DiffDriveBobHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    // Declare both position and velocity states for both wheels
    state_interfaces.emplace_back(hardware_interface::StateInterface(left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(left_wheel_.name, hardware_interface::HW_IF_POSITION, &left_wheel_.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(right_wheel_.name, hardware_interface::HW_IF_POSITION, &right_wheel_.position));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> DiffDriveBobHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    // Declare velocity command interface for both wheels
    command_interfaces.emplace_back(hardware_interface::CommandInterface(left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.command));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.command));

    return command_interfaces;
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
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {

    // Obtain elapsed time
    double delta_seconds = period.seconds();

    // Obtain encoder values
    read_encoder_values(&left_wheel_.encoder_ticks, &right_wheel_.encoder_ticks);


    // Print encoder values to the terminal
    RCLCPP_INFO(logger_, "Left wheel encoder ticks: %d", left_wheel_.encoder_ticks);
    RCLCPP_INFO(logger_, "Right wheel encoder ticks: %d", right_wheel_.encoder_ticks);
    // left_wheel_.encoder_ticks ++;

    // Calculate wheel positions and velocities
    double previous_position = left_wheel_.position;
    left_wheel_.position = left_wheel_.calculate_encoder_angle();
    left_wheel_.velocity = (left_wheel_.position - previous_position) / delta_seconds;

    previous_position = right_wheel_.position;
    right_wheel_.position = right_wheel_.calculate_encoder_angle();
    right_wheel_.velocity = (right_wheel_.position - previous_position) / delta_seconds;

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type bob_base ::DiffDriveBobHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // Map command velocity to value between 0 and 255
    double left_motor_speed = ceil(21.42 * left_wheel_.command);
    double right_motor_speed = ceil(21.42 * right_wheel_.command);

    // RCLCPP_INFO(logger_, "Left wheel command: %f", left_wheel_.command);
    // RCLCPP_INFO(logger_, "Left motor speed: %f", left_motor_speed);


    // Cap max and min velocities
    if (left_motor_speed >= 255)
    {
      left_motor_speed = 255;
    }
    else if (left_motor_speed <= -255)
    {
      left_motor_speed = -255;
    }

    if (right_motor_speed >= 255)
    {
      right_motor_speed = 255;
    }
    else if (right_motor_speed <= -255)
    {
      right_motor_speed = -255;
    }

    // Send commands to motor driver
    set_motor_speeds(pi_int, left_motor_speed, right_motor_speed);

    return hardware_interface::return_type::OK;
  }

} // namespace bob_base

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    bob_base::DiffDriveBobHardware, hardware_interface::SystemInterface)
