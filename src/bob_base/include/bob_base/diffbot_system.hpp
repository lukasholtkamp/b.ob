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
  class DiffDriveBobHardware : public hardware_interface::SystemInterface
  {
    struct Config
    {
      std::string left_wheel_name = "left_wheel";
      std::string right_wheel_name = "right_wheel";
      int enc_ticks_per_rev = 98;
      double loop_rate = 30.0;
    };

  public:
    DiffDriveBobHardware();

    //! 
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    Config config_;

    Wheel left_wheel_;
    Wheel right_wheel_;

    rclcpp::Logger logger_;
  };

} // namespace bob_base

#endif // DIFFDRIVE_BOB__DIFFBOT_SYSTEM_HPP_
