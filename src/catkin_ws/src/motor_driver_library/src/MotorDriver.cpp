#include "MotorDriver.h"

MD::Motor::Motor(int directionPin, int pwmPin, double minSpeed, double maxSpeed) {
    m_directionPin = directionPin;
    m_pwmPin = pwmPin;
    m_minSpeed = minSpeed;
    m_maxSpeed = maxSpeed;
    m_speed = 0.0;
    m_direction = 0;

	// Pin configurations for JetsonGPIO
	GPIO::setmode(GPIO::BOARD); // GPIO::BCM or GPIO::BOARD
	GPIO::setup(32, GPIO::OUT, GPIO::HIGH); // Left motor
	GPIO::setup(33, GPIO::OUT, GPIO::HIGH); // Right motor
	GPIO::setup(31, GPIO::OUT, GPIO::HIGH); // Left direction
	GPIO::setup(35, GPIO::OUT, GPIO::HIGH); // Right direction  
}

// --- Pwm pin ---.
int MD::Motor::getPwmPin(){
    return m_pwmPin;
}
// --- End of Pwm pin ---

// --- Wheel direction ---
void MD::Motor::setDirection(double speed) {
    if (speed >= 0) {
        m_direction = true;
    } else if (speed < 0) {
        m_direction = false; 
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
    if (speed <= m_minSpeed) {
        m_speed = m_minSpeed;
    } else if (speed >= m_maxSpeed) {
        m_speed = m_maxSpeed;
    } else {
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

