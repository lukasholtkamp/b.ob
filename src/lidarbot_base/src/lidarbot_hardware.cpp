#include "lidarbot_base/lidarbot_hardware.hpp"

extern int left_wheel_pulse_count;
extern int right_wheel_pulse_count;
extern int left_wheel_direction;
extern int right_wheel_direction;
extern int* pi_int;

namespace lidarbot_base
{

LidarbotHardware::LidarbotHardware()
    : logger_(rclcpp::get_logger("LidarbotHardware"))
{}

CallbackReturn LidarbotHardware::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
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

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LidarbotHardware::export_state_interfaces()
{
    // Set up a position and velocity interface for each wheel

    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(left_wheel_.name, hardware_interface::HW_IF_POSITION, &left_wheel_.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(right_wheel_.name, hardware_interface::HW_IF_POSITION, &right_wheel_.position));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LidarbotHardware::export_command_interfaces()
{
    // Set up a velocity command for each wheel

    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.command));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.command));

    return command_interfaces;
}

CallbackReturn LidarbotHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Configuring motors and encoders...");

    // Initialize pigpio using GPIO BCM pin numbers
    int pi = pigpio_start(NULL,NULL);
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

    return CallbackReturn::SUCCESS;
}

CallbackReturn LidarbotHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Starting controller ...");

    return CallbackReturn::SUCCESS;
}

CallbackReturn LidarbotHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{   
    RCLCPP_INFO(logger_, "Stopping Controller...");

    return CallbackReturn::SUCCESS;
}

return_type LidarbotHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
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

    return return_type::OK;
}

return_type LidarbotHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{   
    double left_motor_counts_per_loop = left_wheel_.command / left_wheel_.rads_per_tick / config_.loop_rate;
    double right_motor_counts_per_loop = right_wheel_.command / right_wheel_.rads_per_tick / config_.loop_rate;

    // Send commands to motor driver
    set_motor_speeds(left_motor_counts_per_loop, right_motor_counts_per_loop);

    return return_type::OK;
}

} // namespace lidarbot_base

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    lidarbot_base::LidarbotHardware, 
    hardware_interface::SystemInterface)