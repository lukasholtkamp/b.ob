#include "Wheel.hpp"
#include "MotorDriver.hpp"
#include "MotorEncoder.hpp"

// Left wheel callback function
void left_wheel_pulse()
{   
    
}

// Right wheel callback function
void right_wheel_pulse()
{

}

namespace WH{

Wheel::Wheel(const std::string &wheel_name, int ticks_per_rev,int directionPin,int pwmPin, double maxSpeed,int direction,int EncPin, encoderCB_t callback){
    
    std::string name = "";

    setup(wheel_name, ticks_per_rev);

    class MD::Motor Motor(directionPin,pwmPin, maxSpeed,direction);
    class ENC::Encoder Encoder(EncPin, callback);
    double command = 0.0;
    double rads_per_tick = 0.0;
}

void Wheel::setup(const std::string &wheel_name, int ticks_per_rev)
{
    name = wheel_name;
    rads_per_tick = (2*M_PI)/ticks_per_rev;
}

double Wheel::calculate_encoder_angle()
{
    return Encoder.getPulseCount() * rads_per_tick;
}

}