#include "Wheel.hpp"
#include "MotorDriver.hpp"
#include "MotorEncoder.hpp"
#include "MotorAlarm.hpp"

namespace WH{

Wheel::Wheel(const std::string &wheel_name, int ticks_per_rev, double radius, size_t velocity_rolling_window_size, MD::Motor Motor, ENC::Encoder Encoder, ALM::Alarm Alarm){

    name = wheel_name;
    
    Motor = Motor;
    Encoder = Encoder;
    Alarm = Alarm;

    encoder_ticks = 0;
    command = 0;
    old_pos = 0;
    old_velocity = 0;
    radius = radius;
    rads_per_tick = (2*M_PI)/ticks_per_rev;

    velocity_rolling_window_size = velocity_rolling_window_size;
    linear_accumulator = RollingMeanAccumulator(velocity_rolling_window_size);
    angular_accumulator = RollingMeanAccumulator(velocity_rolling_window_size);

    timestamp = 0;

    resetAccumulators();
    timestamp_ = time;
    
}

void Wheel::init(const rclcpp::Time & time)
{
  // Reset accumulators and timestamp:
  resetAccumulators();
  timestamp_ = time;
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

void Wheel::set_speed(double speed){
    command = speed;
}

void Wheel.update(){

    


}

}