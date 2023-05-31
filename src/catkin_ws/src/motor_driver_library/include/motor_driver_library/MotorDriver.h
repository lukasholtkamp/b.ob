#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <JetsonGPIO.h>
#include <stdint.h>

// MD: Motor Driver
namespace MD{
	class Motor {
	private:
		int m_directionPin, m_pwmPin;
		float m_minSpeed, m_maxSpeed;
		float m_speed; 
		bool m_direction;
	public:
		Motor(int directionPin, int pwmPin, float minSpeed, float maxSpeed);

		// --- Pwm pin ---
		int getPwmPin();
		// --- End of pwm pin ---
		
		// --- Wheel direction ---
		
		// Cw: true, CCW: false
		void setDirection(float speed);
		// bool getDirection();
		int getDirectionPin();
		// --- End of wheel direction ---
		
		// --- Control of motor actions ---
		void setSpeed(float speed);
		float getSpeed();
		void setMinSpeed(float minSpeed);
		// int getMinSpeed();
		void setMaxSpeed(float maxSpeed);
		// int getMaxSpeed();
		// void startMotor(int speed);    
		void stop();
	};
}

#endif // MOTOR_DRIVER_H
