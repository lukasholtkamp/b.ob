#include "MotorDriver.h"
#include <chrono>
#include <thread>
#include <wiringPi.h>
#include <softPwm.h>
using namespace std::this_thread; 
using namespace std::chrono; 

MD::Motor::Motor(int directionPin, int pwmPin, double minSpeed, double maxSpeed) {
    m_directionPin = directionPin;
    m_pwmPin = pwmPin;
    m_minSpeed = minSpeed;
    m_maxSpeed = maxSpeed;
    m_speed = 0.0;
    m_direction = 0;
	wiringPiSetup();
	// Pin configurations for JetsonGPIO
	//GPIO::setmode(GPIO::BOARD); // GPIO::BCM or GPIO::BOARD
	//GPIO::setup(32, GPIO::OUT); // Left motor
	pinMode(23, OUTPUT); // Right motor
	softPwmCreate(23,0,100);
	pinMode(26, OUTPUT);
	softPwmCreate(26,0,100);
//	pwmSetMode(PWM_MODE_MS);
//	pwmSetClock(19200);
//	pwmSetRange(500);
	pinMode(22, OUTPUT); // Left direction
	pinMode(24, OUTPUT); // Right direction  
}

// --- Pwm pin ---.
int MD::Motor::getPwmPin(){
    return m_pwmPin;
}
// --- End of Pwm pin ---

// --- Wheel direction ---
void MD::Motor::setDirection(double speed, int pinNumber) {
    if (speed > 0) {
//    	sleep_for(nanoseconds(1000));
        m_direction = true;
	digitalWrite(pinNumber, HIGH);
    } else if (speed <= 0) {
//       	sleep_for(nanoseconds(1000));
        m_direction = false; 
	digitalWrite(pinNumber, LOW);
    }
}

bool MD::Motor::getDirection() {
	return m_direction;
}

int MD::Motor::getDirectionPin(){
    return m_directionPin;
}
// --- End of wheel direction ---

// --- Control of motor actions ---
void MD::Motor::setSpeed(double speed) {
    if (speed < m_minSpeed && speed > m_maxSpeed*-1) {
//    	sleep_for(nanoseconds(1000));
        m_speed = speed*-1;
    } else if (speed <= m_maxSpeed*-1) {
//    	sleep_for(nanoseconds(1000));
    	m_speed = m_maxSpeed;
    } else if (speed >= m_maxSpeed) {
//    	sleep_for(nanoseconds(1000));
        m_speed = m_maxSpeed;
    } else {
//    	sleep_for(nanoseconds(1000));
   		m_speed = speed;
    }
}

double MD::Motor::getSpeed() {
    return m_speed;
}

void MD::Motor::setMinSpeed(double minSpeed) {
    m_minSpeed = minSpeed;
}

// int Motor::getMinSpeed() {return 0;}

void MD::Motor::setMaxSpeed(double maxSpeed) {
    m_maxSpeed = maxSpeed;
}

// int Motor::getMaxSpeed() {return 0;}

void MD::Motor::stop() {
    m_speed = 0.0;
}
// --- End of control of motor actions ---
