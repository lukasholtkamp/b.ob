/**
 * @file MotorEncoder.h
 * @brief Motor Encoder library for the Jetson Nano and the Raspberry Pi
 * @details This library is used to get encoder feedback from the motors of the robot. The library is used to create closed loop velocity control.
 * The library is used by the RunMotor.cpp program, which is used to control the motors of the robot. 
 */
#ifndef MOTOR_ALARM_HPP
#define MOTOR_ALARM_HPP

#include <pigpio.h> //<-- Used to create the PWM pins on the Raspberry Pi
#include <stdint.h> //<-- Used to define the uint8_t type
#include <stdint.h> //<-- Used to define the uint

#define LEFT_ALARM_PIN      22 //<-- Pin number for the direction of the left motor
#define RIGHT_ALARM_PIN     25 //<-- Pin number for the pwm of the left motor


// ME: Motor Encoder
namespace ALM{
  /**
   * @brief Motor class
   * @details This class is used to control the motors of the robot. The class is used to set the speed and direction of the motors.
   */

  class Alarm {

  private:
    int a_Pin; //<-- encoder Pin numbers
    int state;

    void _pulse(int gpio, int level, uint32_t tick);

    static void _pulseEx(int gpio, int level, uint32_t tick, void *user);

  public:
  /**
   * @brief Constructor
   * @details This constructor is used to initialize the motor driver library. The constructor is used to set the pin numbers for the motors, the speed limits and the current speed and direction of the motors.
   * @param directionPin Pin number for the direction of the motor
   * @param pwmPin Pin number for the pwm of the motor
   * @param minSpeed Minimum speed of the motor
   * @param maxSpeed Maximum speed of the motor
   */
    Alarm(int gpio);

    void re_cancel(void);

    int getState();

  };
  }

#endif // MOTOR_DRIVER_H
