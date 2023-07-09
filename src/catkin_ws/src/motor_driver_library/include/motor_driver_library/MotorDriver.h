/**
 * @file MotorDriver.h
 * @brief Motor driver library for the Jetson Nano and the Raspberry Pi
 * @details This library is used to control the motors of the robot. The library is used to set the speed and direction of the motors.
 * The library is used by the RunMotor.cpp program, which is used to control the motors of the robot. 
 */
#ifndef BOB_MOTOR_DRIVER_LIBRARY_MOTORDRIVER_H
#define BOB_MOTOR_DRIVER_LIBRARY_MOTORDRIVER_H

const int left_direction_pin = 22; //<-- Pin number for the direction of the left motor
const int left_pwm_pin = 26; //<-- Pin number for the pwm of the left motor
const int right_direction_pin = 24; //<-- Pin number for the direction of the right motor
const int right_pwm_pin = 23; //<-- Pin number for the pwm of the right motor

// MD: Motor Driver
namespace MD {
	/**
	 * @brief Motor class
	 * @details This class is used to control the motors of the robot. The class is used to set the speed and direction of the motors.
	 */
	class Motor {
    public:
      /**
       * @brief Constructor
       * @details This constructor is used to initialize the motor driver library. The constructor is used to set the pin numbers for the motors, the speed limits and the current speed and direction of the motors.
       * @param direction_pin Pin number for the direction of the motor
       * @param pwm_pin Pin number for the pwm of the motor
       * @param min_speed Minimum speed of the motor
       * @param max_speed Maximum speed of the motor
       */
      Motor(int direction_pin, int pwm_pin, double min_speed, double max_speed);

      int GetPwmPin(); //<-- Returns the pwm pin number
      
      /**
       * @brief Set the Direction object	
       * @param direction 
       * @param pin_number 
       */
      void SetDirection(double direction, int pin_number); 
      bool GetDirection(); //<-- Returns the direction of the motor
      int GetDirectionPin(); //<-- Returns the direction pin number
      
      /**
       * @brief Set the Speed object
       * @param speed 
       */
      void SetSpeed(double speed);
      double GetSpeed(); //<-- Returns the speed of the motor
      void SetMinSpeed(double min_speed); //<-- Sets the minimum speed of the motor
      void SetMaxSpeed(double max_speed); //<-- Sets the maximum speed of the motor    
      void Stop(); //<-- Stops the motor

    private:
      int direction_pin_, pwm_pin_; //<-- Pin numbers
      double min_speed_, max_speed_; //<-- Speed limits
      double speed_; //<-- Current speed
      bool direction_; //<-- Current direction
	};
}

#endif // BOB_MOTOR_DRIVER_LIBRARY_MOTORDRIVER_H
