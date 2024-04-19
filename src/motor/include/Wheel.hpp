
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
    std::string name;
    class MD::Motor Motor;
    class ENC::Encoder Encoder;
    double command;
    double rads_per_tick;

    Wheel(const std::string &wheel_name, int ticks_per_rev,int directionPin,int pwmPin, double maxSpeed,int direction,int EncPin, encoderCB_t callback);

    void setup(const std::string &wheel_name, int ticks_per_rev);

    double calculate_encoder_angle();

};

}

#endif