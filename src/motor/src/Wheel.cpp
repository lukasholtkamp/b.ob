#include "Wheel.hpp"
#include "MotorDriver.hpp"
#include "MotorEncoder.hpp"
#include "MotorAlarm.hpp"

namespace WH{

Wheel::Wheel(const std::string &wheel_name, int ticks_per_rev, double radius, size_t velocity_rolling_window_size, MD::Motor Motor_obj, ENC::Encoder Encoder_obj, ALM::Alarm Alarm_obj){

    setup(wheel_name,ticks_per_rev,radius,velocity_rolling_window_size,Motor_obj, Encoder_obj,Alarm_obj);
    
}

// void Wheel::init(const rclcpp::Time & time)
// {
//   // Reset accumulators and timestamp:
//   resetAccumulators();
//   timestamp_ = time;
// }
void Wheel::setup(const std::string &wheel_name, int ticks_per_rev, double radius, size_t velocity_rolling_window_size, MD::Motor Motor_obj, ENC::Encoder Encoder_obj, ALM::Alarm Alarm_obj)
{
    name = wheel_name;
    
    Motor = Motor_obj;
    Encoder = Encoder_obj;
    Alarm = Alarm_obj;

    encoder_ticks = 0;
    command = 0;
    position = 0;
    velocity = 0;
    old_pos = 0;
    old_velocity = 0;
    radius = radius;
    rads_per_tick = (2*M_PI)/ticks_per_rev;

    Alarm_state = Alarm_obj.getState();

    velocity_rolling_window_size = velocity_rolling_window_size;
    // linear_accumulator = RollingMeanAccumulator(velocity_rolling_window_size);
    // angular_accumulator = RollingMeanAccumulator(velocity_rolling_window_size);

    // timestamp = 0;
}

double Wheel::calculate_encoder_angle()
{
    return encoder_ticks * rads_per_tick;
}

void Wheel::set_speed(double speed){
    command = speed;
    Motor.setSpeed(speed);
}

void Wheel::update(){

    // velocity = (Encoder.getMotorSpeed()*2*M_PI*radius) / 60.0;
    Alarm_state = Alarm.getState();

}

}