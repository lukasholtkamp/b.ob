// Based on: https://github.com/TheNoobInventor/lidarbot/blob/main/lidarbot_base/include/lidarbot_base/wheel.hpp
// Date of Retrieval: 17.05.2024

#ifndef _DIFFDRIVE_BOB__WHEEL_H_
#define _DIFFDRIVE_BOB__WHEEL_H_

#include <string>
#include <cmath>

class Wheel
{
public:
  std::string name = "";
  int encoder_ticks = 0;
  double command = 0.0;
  double position = 0.0;
  double velocity = 0.0;
  double rads_per_tick = 0.0;

  Wheel() = default;

  //! Wheel Constructor
  Wheel(const std::string &wheel_name, int ticks_per_rev);

  //! Function to assign variables
  void setup(const std::string &wheel_name, int ticks_per_rev);

  //! Function to calculate the encoder angle in radians
  double calculate_encoder_angle();
};

#endif