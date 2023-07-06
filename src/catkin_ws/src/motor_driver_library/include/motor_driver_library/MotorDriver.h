/**
 * @file MotorDriver.h
 * @brief Motor driver library for the Jetson Nano and the Raspberry Pi
 * @details This library is used to control the motors of the robot. The library is used to set the speed and direction of the motors.
 * The library is used by the RunMotor.cpp program, which is used to control the motors of the robot. 
 */
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

//#include <JetsonGPIO.h>
#include <stdint.h>
#include <wiringPi.h>
// MD: Motor Driver
namespace MD{
	/**
	 * @brief Motor class
	 * @details This class is used to control the motors of the robot. The class is used to set the speed and direction of the motors.
	 */
	class Motor {
	private:
		int m_directionPin, m_pwmPin; //<-- Pin numbers
		double m_minSpeed, m_maxSpeed; //<-- Speed limits
		double m_speed; //<-- Current speed
		bool m_direction; //<-- Current direction
	public:
		/**
		 * @brief Constructor
		 * @details This constructor is used to initialize the motor driver library. The constructor is used to set the pin numbers for the motors, the speed limits and the current speed and direction of the motors.
		 * @param directionPin Pin number for the direction of the motor
		 * @param pwmPin Pin number for the pwm of the motor
		 * @param minSpeed Minimum speed of the motor
		 * @param maxSpeed Maximum speed of the motor
		 */
		Motor(int directionPin, int pwmPin, double minSpeed, double maxSpeed);

		int getPwmPin(); //<-- Returns the pwm pin number
		
		/**
		 * @brief Set the Direction object	
		 * 
		 * @param direction 
		 * @param pinNumber 
		 */
		void setDirection(double direction,int pinNumber); 
		bool getDirection(); //<-- Returns the direction of the motor
		int getDirectionPin(); //<-- Returns the direction pin number
		
		/**
		 * @brief Set the Speed object
		 * 
		 * @param speed 
		 */
		void setSpeed(double speed);
		double getSpeed(); //<-- Returns the speed of the motor
		void setMinSpeed(double minSpeed); //<-- Sets the minimum speed of the motor
		// int getMinSpeed();
		void setMaxSpeed(double maxSpeed); //<-- Sets the maximum speed of the motor
		// int getMaxSpeed();
		// void startMotor(int speed);    
		void stop(); //<-- Stops the motor
	};
}

#endif // MOTOR_DRIVER_H
