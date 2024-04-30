#include "Wheel.hpp"
#include "MotorDriver.hpp"
#include "MotorEncoder.hpp"
#include "MotorAlarm.hpp"

namespace WH{

Wheel::Wheel(const std::string &wheel_name, int ticks_per_rev, double wheel_radius, MD::Motor& Motor_obj, ENC::Encoder& Encoder_obj, ALM::Alarm& Alarm_obj)
    :
    name(wheel_name),
    Motor(Motor_obj),
    Encoder(Encoder_obj),
    Alarm(Alarm_obj),
    encoder_ticks(0),
    command(0),
    u(0),
    position(0),
    velocity(0),
    old_position(0),
    old_velocity(0),
    old_e(0),
    sum_e(0),
    radius(wheel_radius),
    rads_per_tick((2*M_PI)/ticks_per_rev)
    {}


double Wheel::calculate_encoder_angle()
{
    return encoder_ticks * rads_per_tick;
}

void Wheel::set_speed(double speed){
    command = speed;
}

void Wheel::update(){

    auto time_stamp_start = std::chrono::high_resolution_clock::now();

    old_position = position;
    old_velocity = velocity;

    position = calculate_encoder_angle()*radius;
    
    auto time_stamp_end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> diff = time_stamp_end - time_stamp_start;

    velocity = 2* radius * M_PI * (Motor.getSpeed()/60);

    u = PID(command-velocity,diff.count());

    if(u>255){
        u=255;
    }

    Motor.setSpeed(u);

}

double Wheel::PID(double e,double dt){
    sum_e += e*dt;
    double dedt = (old_e - e)/dt;
    return KP*e + KI*sum_e + KD*dedt;
}

}