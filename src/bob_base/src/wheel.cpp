// Based on: https://github.com/TheNoobInventor/lidarbot/blob/main/lidarbot_base/src/wheel.cpp
// Date of Retrieval: 17.05.2024

#include "wheel.hpp"

// Wheel constructor
Wheel::Wheel(const std::string &wheel_name, int ticks_per_rev, double wheel_r)
{
    setup(wheel_name, ticks_per_rev, wheel_r);
}

// Wheel setup function to assign values
void Wheel::setup(const std::string &wheel_name, int ticks_per_rev, double wheel_r)
{
    name = wheel_name;
    rads_per_tick = (2 * M_PI) / ticks_per_rev;
    wheel_radius = wheel_r;
}

// Calculate the encoder angle
double Wheel::calculate_encoder_angle()
{
    return encoder_ticks * rads_per_tick;
}