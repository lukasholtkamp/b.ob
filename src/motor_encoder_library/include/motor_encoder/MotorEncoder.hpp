/**
 * @file MotorEncoder.h
 * @brief Motor Encoder library for the Jetson Nano and the Raspberry Pi
 * @details This library is used to get encoder feedback from the motors of the robot. The library is used to create closed loop velocity control.
 * The library is used by the RunMotor.cpp program, which is used to control the motors of the robot. 
 */
#ifndef MOTOR_ENCODER_HPP
#define MOTOR_ENCODER_HPP

#include <pigpio.h> //<-- Used to create the PWM pins on the Raspberry Pi
#include <stdint.h> //<-- Used to define the uint8_t type
#include <stdint.h> //<-- Used to define the uint

#define LEFT_ENCODER_PIN      3 //<-- Pin number for the direction of the left motor
#define RIGHT_ENCODER_PIN     24 //<-- Pin number for the pwm of the left motor

typedef void (*encoderCB_t)(int);

// ME: Motor Encoder
namespace ENC{
  /**
   * @brief Motor class
   * @details This class is used to control the motors of the robot. The class is used to set the speed and direction of the motors.
   */

  class Encoder {

  private:
    int e_Pin; //<-- encoder Pin numbers
    double _weighting,_new,_old;
    u_int32_t _high_tick,_period,_high;
    uint pos;

    encoderCB_t mycallback;

    void _pulse(int gpio, int level, uint32_t tick);

    static void _pulseEx(int gpio, int level, uint32_t tick, void *user);

    u_int32_t _tick_diff(u_int32_t o_tick, u_int32_t c_tick);

  
  public:
  /**
   * @brief Constructor
   * @details This constructor is used to initialize the motor driver library. The constructor is used to set the pin numbers for the motors, the speed limits and the current speed and direction of the motors.
   * @param directionPin Pin number for the direction of the motor
   * @param pwmPin Pin number for the pwm of the motor
   * @param minSpeed Minimum speed of the motor
   * @param maxSpeed Maximum speed of the motor
   */
    Encoder(int gpio, encoderCB_t callback);

    void re_cancel(void);

    double getFreq() const;

    double getMotorSpeed() const;

    double getDutyCycle() const;

  };
  }

#endif // MOTOR_DRIVER_H
