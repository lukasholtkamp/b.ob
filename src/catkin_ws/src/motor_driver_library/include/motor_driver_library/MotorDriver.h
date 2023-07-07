#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

//#include <JetsonGPIO.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <stdint.h>

#define LEFT_DIRECTION_PIN      22
#define LEFT_PWM_PIN            26
#define RIGHT_DIRECTION_PIN     24
#define RIGHT_PWM_PIN           23

// MD: Motor Driver
namespace MD{
	class Motor {
	private:
		int mDirectionPin, mPwmPin;
		double mMinSpeed, mMaxSpeed;
		double mSpeed;
		bool mDirection;
	public:
		Motor(int directionPin, int pwmPin, double minSpeed, double maxSpeed);

		// --- Pwm pin ---
		int getPwmPin();
		// --- End of pwm pin ---

		// --- Wheel direction ---

		// Cw: !0, CCW: 0
		void setDirection(double direction,int directionPin);
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
