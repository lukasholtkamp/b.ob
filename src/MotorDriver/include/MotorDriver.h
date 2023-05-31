#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <JetsonGPIO.h>
#include <stdint.h>

// MD: Motor Driver
namespace MD{
	class Motor {
	private:
		int m_directionPin, m_pwmPin;
		int m_minSpeed, m_maxSpeed;
		int16_t m_speed; 
		bool m_direction;
	public:
		Motor(int directionPin, int pwmPin, int minSpeed, int maxSpeed);

		// --- Pwm pin ---
		int getPwmPin();
		// --- End of pwm pin ---
		
		// --- Wheel direction ---
		
		// Cw: true, CCW: false
		void setDirection(int16_t speed);
		// bool getDirection();
		int getDirectionPin();
		// --- End of wheel direction ---
		
		// --- Control of motor actions ---
		void setSpeed(int16_t speed);
		int16_t getSpeed();
		void setMinSpeed(int minSpeed);
		// int getMinSpeed();
		void setMaxSpeed(int maxSpeed);
		// int getMaxSpeed();
		// void startMotor(int speed);    
		void stop();
	};
}

#endif // MOTOR_DRIVER_H
