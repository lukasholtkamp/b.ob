// Based on: https://github.com/ros2/teleop_twist_joy/blob/rolling/src/teleop_node.cpp
// Date of Retrieval: 17.05.2024

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "bob_teleop/teleop_twist_joy.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_unique<bob_teleop::TeleopTwistJoy>(rclcpp::NodeOptions()));

  rclcpp::shutdown();

  return 0;
}