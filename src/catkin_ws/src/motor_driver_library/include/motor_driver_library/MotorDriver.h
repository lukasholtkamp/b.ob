#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <JetsonGPIO.h>
#include <stdint.h>

// MD: Motor Driver
namespace MD{
	class Motor {
	private:
		int m_directionPin, m_pwmPin;
		double m_minSpeed, m_maxSpeed;
		double m_speed; 
		bool m_direction;
	public:
		Motor(int directionPin, int pwmPin, double minSpeed, double maxSpeed);

		// --- Pwm pin ---
		int getPwmPin();
		// --- End of pwm pin ---
		
		// --- Wheel direction ---
		
		// Cw: !0, CCW: 0
		void setDirection(int direction);
		bool getDirection();
		int getDirectionPin();
		// --- End of wheel direction ---
		
		// --- Control of motor actions ---
		void setSpeed(double speed);
		double getSpeed();
		void setMinSpeed(double minSpeed);
		// int getMinSpeed();
		void setMaxSpeed(double maxSpeed);
		// int getMaxSpeed();
		// void startMotor(int speed);    
		void stop();
	};
}

#endif // MOTOR_DRIVER_H
