/**
 * @file MotorDriver.h
 * @brief Motor driver library for the Raspberry Pi
 * @details This library is used to control the motors of the robot. The library is used to set the speed of the motors.
 */
#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include <pigpio.h> //<-- Used to create the PWM pins on the Raspberry Pi
#include <stdint.h> //<-- Used to define the uint
#include <string>
#include <math.h>

#define LEFT_DIRECTION_PIN      6 //<-- Pin number for the direction of the left motor
#define LEFT_PWM_PIN            13 //<-- Pin number for the pwm of the left motor
#define RIGHT_DIRECTION_PIN     19 //<-- Pin number for the direction of the right motor
#define RIGHT_PWM_PIN           12 //<-- Pin number for the pwm of the right motor

#define MAX_SPEED               255 //<-- The maximum speed of the robot
#define MIN_SPEED               13 //<-- The maximum speed of the robot

#define CCW 1 //<-- Value to be written to direction pin for motor to go counterclockwise
#define CW 0 //<-- Value to be written to direction pin for motor to go clockwise

// MD: Motor Driver
namespace MD{
  /**
   * @brief Motor class
   * @details This class is used to control the motors of the robot. The class is used to set the speed and direction of the motors.
   */

  class Motor {

  private:
    int m_DirectionPin=0;
    int m_PwmPin=0; //<-- Pin numbers
    int m_MaxSpeed=0; //<-- Speed limits
    int m_MinSpeed=0; //<-- Speed limits
    double m_Speed=0; //<-- Current speed
    bool m_Direction=0; //<-- Current direction
    bool m_FDirection=0; //<-- Direction considered as Forward
    u_int PWM_Range=0; //<-- PWM range. Default is 255 but a bigger range can be chosen to get finer speed tuning

  public:
  /**
   * @brief Constructor
   * @details This constructor is used to initialize the motor driver library. The constructor is used to set the pin numbers for the motors, the speed limits and the current speed and direction of the motors.
   * @param directionPin Pin number for the direction of the motor
   * @param pwmPin Pin number for the pwm of the motor
   * @param maxSpeed Maximum speed of the motor
   */
    Motor() = default;

    Motor(int directionPin, int pwmPin, double maxSpeed, int direction);

    int getPwmPin() const; //<-- Returns the pwm pin number

    u_int getPWMRange() const; //<-- Returns the pwm range

    void setPWMRange(uint range); //<-- Sets the pwm range

    void switchDirection(); //<-- Changes motor direction 

    std::string getDirection() const; //<-- Returns the direction of the motor

    int getDirectionPin() const; //<-- Returns the direction pin number

    void setSpeed(double speed); //<-- Sets motor speed

    double getSpeed() const; //<-- Returns the speed of the motor

    void setMaxSpeed(double maxSpeed); //<-- Sets the maximum speed of the motor

    void stop(); //<-- Stops the motor
  };
}

#endif // MOTOR_DRIVER_H