
#ifndef WHEEL_HPP
#define WHEEL_HPP

#include <string>
#include <cmath>
#include <ctime>
#include <chrono>

#include "MotorDriver.hpp"
#include "MotorEncoder.hpp"
#include "MotorAlarm.hpp" 

namespace WH{

class Wheel
{   
  public:
    MD::Motor& Motor;
    ENC::Encoder& Encoder;
    ALM::Alarm& Alarm;

    std::string name;
    int encoder_ticks = 0;
    double command = 0;
    double u = 0;
    double position = 0;
    double velocity = 0;
    double old_position = 0;
    double old_velocity = 0;
    double old_e = 0;
    double sum_e = 0;
    double radius = 0;
    double rads_per_tick = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    Wheel(const std::string &wheel_name, int ticks_per_rev, double wheel_radius, MD::Motor& Motor_obj, ENC::Encoder& Encoder_obj, ALM::Alarm& Alarm_obj);

    double calculate_encoder_angle();

    void update();

    void set_speed(double speed);

    void set_PID(int Kp_gain, int Ki_gain, int Kd_gain);

    double PID(double error, double dt);

};

}

#endif