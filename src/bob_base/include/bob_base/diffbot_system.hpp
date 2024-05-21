// Based on: https://github.com/TheNoobInventor/lidarbot/blob/main/lidarbot_base/include/lidarbot_base/lidarbot_hardware.hpp
// Date of Retrieval: 17.05.2024

#ifndef DIFFDRIVE_BOB__DIFFBOT_SYSTEM_HPP_
#define DIFFDRIVE_BOB__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "motor.hpp"
#include "wheel.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace bob_base
{
  //! This class was created using ros2_control concepts. <br>
  //! More info: https://control.ros.org/rolling/index.html <br>

  //! This class uses concepts of Lifecycle States. <br>
  //! More info: https://design.ros2.org/articles/node_lifecycle.html
  class DiffDriveBobHardware : public hardware_interface::SystemInterface
  {
    //! Configuration struct with name and motor parameter information
    struct Config
    {
      std::string left_wheel_name = "left_wheel";
      std::string right_wheel_name = "right_wheel";
      int enc_ticks_per_rev = 98;
      double loop_rate = 30.0;
    };

  public:
    DiffDriveBobHardware();

    //! Function that gets called when the startup state switches to the unconfigured state. <br>
    //! Configuration parameters and the wheel objects are setup in this function.
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    //! Function to declare which state interfaces exist
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    //! Function to declare which command interfaces exist
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    //! Function that gets called when the unconfigured state switches to the inactive state. <br>
    //! The gpio pins for the motor driver, encoder and alarm are setup in this function.
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    //! Function that gets called when the inactive state switches to the active state. <br>
    //! Unused but can be used to engage actuators.
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    //! Function that gets called when the active state switches to the inactive state. <br>
    //! Unused but can be used to disengage actuators.
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    //! Function that reads and updates to the state interfaces.
    return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    //! Function that write to the command interfaces.
    return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    //! Integer identifying the Raspberry pi which is being run
    int pi_int;

    //! Configuration object with all the necessary parameters
    Config config_;

    //! Left Wheel object
    Wheel left_wheel_;

    //! Right Wheel object
    Wheel right_wheel_;

    //! ROS Logger for displaying messages in terminal
    rclcpp::Logger logger_;
  };

} // namespace bob_base

#endif // DIFFDRIVE_BOB__DIFFBOT_SYSTEM_HPP_
