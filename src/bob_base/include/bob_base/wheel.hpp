// Based on: https://github.com/TheNoobInventor/lidarbot/blob/main/lidarbot_base/include/lidarbot_base/wheel.hpp
// Date of Retrieval: 17.05.2024

#ifndef _DIFFDRIVE_BOB__WHEEL_H_
#define _DIFFDRIVE_BOB__WHEEL_H_

#include <string>
#include <cmath>

class Wheel
{
public:
  //! Name that can be found in the URDF description of B.ob
  std::string name = "";
  //! Counter to track 
  int encoder_ticks = 0;
  //! Target velocity of wheel
  double command = 0.0;
  //! Position of wheel according to encoder ticks
  double position = 0.0;
  //! Velocity of wheel according to encoder ticks
  double velocity = 0.0;
  //! The amount radian change with one encoder tick
  double rads_per_tick = 0.0;
  //! Wheel radius
  double wheel_radius = 0.0;

  Wheel() = default;

  //! Wheel Constructor
  Wheel(const std::string &wheel_name, int ticks_per_rev, double wheel_r);

  //! Function to assign variables
  void setup(const std::string &wheel_name, int ticks_per_rev,double wheel_r);

  //! Function to calculate the encoder angle in radians
  double calculate_encoder_angle();
};

#endif