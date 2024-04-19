
#ifndef WHEEL_HPP
#define WHEEL_HPP

#include <string>
#include <cmath>

#include "MotorDriver.hpp"
#include "MotorEncoder.hpp"

namespace WH{

class Wheel
{   
  public:
    std::string name = "";
    class MD::Motor;
    class ENC::Encoder;

    double command = 0.0;
    double rads_per_tick = 0.0;

    Wheel() = default;

    Wheel(const std::string &wheel_name, int ticks_per_rev);

    void setup(const std::string &wheel_name, int ticks_per_rev);

    double calculate_encoder_angle();

};

}

#endif