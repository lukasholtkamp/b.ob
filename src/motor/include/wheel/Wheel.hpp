
#ifndef WHEEL_HPP
#define WHEEL_HPP

#include <string>
#include <cmath>

#include "rclcpp/time.hpp"
#include "rcppmath/rolling_mean_accumulator.hpp"

#include "MotorDriver.hpp"
#include "MotorEncoder.hpp"
#include "MotorAlarm.hpp"

namespace WH{

class Wheel
{   
  public:
    std::string name;

    class MD::Motor Motor;
    class ENC::Encoder Encoder;
    class ALM::Alarm Alarm;

    int encoder_ticks;
    double command;
    double position;
    double velocity;
    double old_pos;
    double old_velocity;
    double radius;
    double rads_per_tick;
    size_t velocity_rolling_window_size;
    RollingMeanAccumulator linear_accumulator;
    RollingMeanAccumulator angular_accumulator;

    rclcpp::Time timestamp_;

    Wheel(const std::string &wheel_name, int ticks_per_rev, double radius, size_t velocity_rolling_window_size, MD::Motor Motor, ENC::Encoder Encoder, ALM::Alarm Alarm);

    void init(const rclcpp::Time & time);
    
    double calculate_encoder_angle();

    void update();

    void set_speed(double speed);

};

}

#endif