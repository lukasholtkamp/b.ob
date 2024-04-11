/**
 * @file MotorDriver.h
 * @brief Motor driver library for the Jetson Nano and the Raspberry Pi
 * @details This library is used to control the motors of the robot. The library is used to set the speed and direction of the motors.
 * The library is used by the RunMotor.cpp program, which is used to control the motors of the robot. 
 */
#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include <pigpio.h> //<-- Used to create the PWM pins on the Raspberry Pi
#include <stdint.h> //<-- Used to define the uint8_t type
#include <cmath> //<-- Used to check when there is a change in direction
#include <string>

#define LEFT_DIRECTION_PIN      6 //<-- Pin number for the direction of the left motor
#define LEFT_PWM_PIN            13 //<-- Pin number for the pwm of the left motor
#define RIGHT_DIRECTION_PIN     19 //<-- Pin number for the direction of the right motor
#define RIGHT_PWM_PIN           12 //<-- Pin number for the pwm of the right motor

#define CCW 1
#define CW 0

// MD: Motor Driver
namespace MD{
  /**
   * @brief Motor class
   * @details This class is used to control the motors of the robot. The class is used to set the speed and direction of the motors.
   */

  class Motor {

  private:
    int m_DirectionPin, m_PwmPin; //<-- Pin numbers
    double m_MinSpeed, m_MaxSpeed; //<-- Speed limits
    double m_Speed; //<-- Current speed
    bool m_Direction; //<-- Current direction
    bool m_FDirection; //<-- Direction considered as Forward

  public:
  /**
   * @brief Constructor
   * @details This constructor is used to initialize the motor driver library. The constructor is used to set the pin numbers for the motors, the speed limits and the current speed and direction of the motors.
   * @param directionPin Pin number for the direction of the motor
   * @param pwmPin Pin number for the pwm of the motor
   * @param minSpeed Minimum speed of the motor
   * @param maxSpeed Maximum speed of the motor
   */
    Motor(int directionPin, int pwmPin, double minSpeed, double maxSpeed, int direction);

    int getPwmPin() const; //<-- Returns the pwm pin number

    double LinearAndAngularVelocities(double linearVelocityX, double angularVelocityZ);

    /**
     * @brief Set the Direction object
     *
     * @param direction
     * @param pinNumber
     */
    void switchDirection(); 

    std::string getDirection() const; //<-- Returns the direction of the motor

    int getDirectionPin() const; //<-- Returns the direction pin number

    /**
     * @brief Set the Speed object
     * 
     * @param speed 
     */
    void setSpeed(double speed);

    double getSpeed() const; //<-- Returns the speed of the motor

    void setMinSpeed(double minSpeed); //<-- Sets the minimum speed of the motor

    void setMaxSpeed(double maxSpeed); //<-- Sets the maximum speed of the motor

    void stop(); //<-- Stops the motor
  };
}

#endif // MOTOR_DRIVER_H
