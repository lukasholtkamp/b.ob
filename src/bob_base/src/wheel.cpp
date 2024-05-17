#include "wheel.hpp"

// Wheel constructor
Wheel::Wheel(const std::string &wheel_name, int ticks_per_rev)
{
    setup(wheel_name, ticks_per_rev);
}

// Wheel setup function to assign values
void Wheel::setup(const std::string &wheel_name, int ticks_per_rev)
{
    name = wheel_name;
    rads_per_tick = (2 * M_PI) / ticks_per_rev;
}

// Calculate the encoder angle
double Wheel::calculate_encoder_angle()
{
    return encoder_ticks * rads_per_tick;
}