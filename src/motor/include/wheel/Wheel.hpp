
#ifndef WHEEL_HPP
#define WHEEL_HPP

#include <string>
#include <cmath>

#include "MotorDriver.hpp"
#include "MotorEncoder.hpp"
#include "MotorAlarm.hpp"

#define KP  1
#define KI  0   
#define KD  0    

namespace WH{

class Wheel
{   
  public:
    std::string name;

    class MD::Motor Motor;
    class ENC::Encoder Encoder;
    class ALM::Alarm Alarm;

    int encoder_ticks = 0;
    double command = 0;
    double position = 0;
    double velocity = 0;
    double old_pos = 0;
    double old_velocity = 0;
    double radius = 0;
    double rads_per_tick = 0;
    size_t velocity_rolling_window_size = 0;
    
    // RollingMeanAccumulator linear_accumulator;
    // RollingMeanAccumulator angular_accumulator;

    // rclcpp::Time timestamp_;

    Wheel() = default;

    Wheel(const std::string &wheel_name, int ticks_per_rev, double radius, size_t velocity_rolling_window_size, MD::Motor Motor, ENC::Encoder Encoder, ALM::Alarm Alarm);

    void setup(const std::string &wheel_name, int ticks_per_rev, double radius, size_t velocity_rolling_window_size, MD::Motor Motor, ENC::Encoder Encoder, ALM::Alarm Alarm);

    // void init(const rclcpp::Time & time);
    
    double calculate_encoder_angle();

    void update();

    void set_speed(double speed);
};

}

#endif