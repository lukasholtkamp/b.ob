/**
 * @file MotorEncoder.hpp
 * @brief Motor Encoder library for the the Raspberry Pi
 * @details This library is used to get encoder feedback from the motors of the robot. The library is used to create closed loop velocity control.
 */
#ifndef MOTOR_ENCODER_HPP
#define MOTOR_ENCODER_HPP

#include <pigpiod_if2.h> //<-- Used to create the PWM pins on the Raspberry Pi
#include <stdint.h> //<-- Used to define the uint8_t type
#include <stdint.h> //<-- Used to define the uint
#include <bits/stdc++.h>

#include "Rolling_median.hpp"

#define LEFT_ENCODER_PIN      23 //<-- Pin number for the left motor encoder 
#define RIGHT_ENCODER_PIN     24 //<-- Pin number for the right motor encoder

typedef void (*encoderCB_t)(int); //<-- callback function for tracking number of pulses

// ME: Motor Encoder
namespace ENC{
  /**
   * @brief Motor Encoder class
   * @details This class is used to read the encoders of the motors of the robot. The class is used to read the speed of the motors.
   */

  class Encoder {

  private:
    int pi = 0;
    int e_Pin = 0; //<-- encoder Pin numbers
    double _weighting = 0;
    double _new = 0;
    double _old = 0; //<-- Weighting for new and old reading for smoothing
    int ticks_per_rev = 0;
    u_int32_t _high_tick = 0;
    u_int32_t _period = 0; //<-- Variables for timing of pulse signal
    
    rolling_median mperiod = rolling_median(10);

    u_int32_t med_period = 0;

    encoderCB_t mycallback = 0; //<-- callback function for tracking number of pulses

    void _pulse(int pi, u_int gpio, u_int level, uint32_t tick); //<-- function gets called everytime there is a change on the e_Pin

    static void _pulseEx(int pi, u_int gpio, u_int level, uint32_t tick, void *user); //<-- function gets called everytime there is a change on the e_Pin

    u_int32_t _tick_diff(u_int32_t o_tick, u_int32_t c_tick);//<-- function to get difference between to times

  
  public:
  /**
   * @brief Constructor
   * @details This constructor is used to initialize the motor driver library. The constructor is used to set the pin numbers for the motors, the speed limits and the current speed and direction of the motors.
   * @param gpio Pin number for the encoder signal from the motor
   * @param callback callback function for calculating number of pulses
   */
    Encoder() = default;

    Encoder(int pi_var,int gpio, encoderCB_t callback, int tpr);

    void re_cancel(void);//<--Cancels the reader and releases resources.

    double getFreq() const; //<--Calculates frequency from period

    double getMotorSpeed() const; //<--Calculates RPM from frequency

    u_int32_t getPeriod() const; //<--Gets the period of the signal

  };
  }

#endif // MOTOR_ENCODER_HPP