// Based on: https://github.com/ros2/teleop_twist_joy/blob/rolling/include/teleop_twist_joy/teleop_twist_joy.hpp
// Date of Retrieval: 17.05.2024

#ifndef TELEOP_TWIST_JOY__TELEOP_TWIST_JOY_HPP_
#define TELEOP_TWIST_JOY__TELEOP_TWIST_JOY_HPP_

#include <rclcpp/rclcpp.hpp>
#include "bob_teleop/bob_teleop_export.h"

namespace bob_teleop
{

  /**
   * Class implementing a basic Joy -> Twist translation.
   */
  class BOB_TELEOP_EXPORT TeleopTwistJoy : public rclcpp::Node
  {
  public:
    explicit TeleopTwistJoy(const rclcpp::NodeOptions &options);

    virtual ~TeleopTwistJoy();

  private:
    struct Impl;
    //! A struct with all the parameters for teleoperation
    Impl *pimpl_;

    //! Callback function handle for the joy_sub subscriber to listen to the joy topic
    OnSetParametersCallbackHandle::SharedPtr callback_handle;
  };

} // namespace bob_teleop

#endif // BOB_TELEOP_TELEOP_TWIST_JOY_HPP_