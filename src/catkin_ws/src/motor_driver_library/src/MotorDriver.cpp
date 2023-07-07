#include "MotorDriver.h"

MD::Motor::Motor(int directionPin,int pwmPin, double minSpeed, double maxSpeed) {
	mDirectionPin = directionPin;
	mPwmPin = pwmPin;
	mMinSpeed = minSpeed;
	mMaxSpeed = maxSpeed;
	mSpeed = 0.0;
	mDirection = 0;
	wiringPiSetup();
	// Pin configurations for JetsonGPIO
	//GPIO::setmode(GPIO::BOARD); // GPIO::BCM or GPIO::BOARD
	//GPIO::setup(32, GPIO::OUT); // Left PWM
	//GPIO::setup(31, GPIO::OUT); // Left direction
	//GPIO::setup(33, GPIO::OUT); // Right PWM
	//GPIO::setup(35, GPIO::OUT); // Right direction

	// --- Left Motor ---
	pinMode(LEFT_PWM_PIN, OUTPUT);		// Left PWM
	softPwmCreate(LEFT_PWM_PIN,0,100);
	pinMode(LEFT_DIRECTION_PIN, OUTPUT); 		// Left direction
	// --- Right Motor ---
	pinMode(RIGHT_PWM_PIN, OUTPUT);		// Right PWM
	softPwmCreate(RIGHT_PWM_PIN,0,100);
	pinMode(RIGHT_DIRECTION_PIN, OUTPUT); 		// Right direction
}

// --- Pwm pin ---.
int MD::Motor::getPwmPin(){
	return mPwmPin;
}
// --- End of Pwm pin ---

// --- Wheel direction ---
void MD::Motor::setDirection(double speed, int directionPin) {
	if (speed > mMinSpeed) {
		mDirection = true;
		digitalWrite(directionPin, HIGH);
	} else if (speed <= mMinSpeed) {
		mDirection = false;
		digitalWrite(directionPin, LOW);
	}
}

bool MD::Motor::getDirection() {
	return mDirection;
}

int MD::Motor::getDirectionPin(){
	return mDirectionPin;
}
// --- End of wheel direction ---

// --- Control of motor actions ---
void MD::Motor::setSpeed(double speed) {
	if (speed < mMinSpeed && speed > mMaxSpeed*-1) {
        	mSpeed = speed*-1;
	}
	else if (speed <= mMaxSpeed*-1) {
    		mSpeed = mMaxSpeed;
    	}
	else if (speed >= mMaxSpeed) {
        	mSpeed = mMaxSpeed;
	}
	else {
		mSpeed = speed;
    	}
}

double MD::Motor::getSpeed() {
	return mSpeed;
}

void MD::Motor::setMinSpeed(double minSpeed) {
	mMinSpeed = minSpeed;
}

// int Motor::getMinSpeed() {return 0;}

void MD::Motor::setMaxSpeed(double maxSpeed) {
	mMaxSpeed = maxSpeed;
}

// int Motor::getMaxSpeed() {return 0;}

void MD::Motor::stop() {
	mSpeed = 0.0;
}
// --- End of control of motor actions ---
