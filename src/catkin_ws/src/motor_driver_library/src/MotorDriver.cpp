/**
 * @file MotorDriver.cpp
 * @brief 
 * @version 0.1
 * @date 2023-07-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "MotorDriver.h" 
#include <chrono> //<-- Used to measure time
#include <thread> //<-- Used to make the program sleep
#include <wiringPi.h>
#include <softPwm.h>

using namespace std::this_thread; 
using namespace std::chrono; 

/**
 * @brief Construct a new MD::Motor::Motor object
 * 
 * @param directionPin 
 * @param pwmPin 
 * @param minSpeed 
 * @param maxSpeed 
 */
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

/// @brief Returns the pwm pin of the motor
/// @return int 
int MD::Motor::getPwmPin(){
    return m_pwmPin;
}

/**
 * @brief Set the direction of the object
 * 
 * @param speed 
 * @param pinNumber 
 * @details This function is used to set the direction of the motor. If the speed is positive, the direction is forward. If the speed is negative, the direction is backward.
 */
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

/**
 * @brief Get the direction of the object
 * 
 * @return true 
 * @return false 
 */
bool MD::Motor::getDirection() {
	return m_direction;
}

/// @brief Returns the direction pin of the motor
int MD::Motor::getDirectionPin(){
    return m_directionPin;
}

/**
 * @brief Set the Speed of the object
 * 
 * @param speed 
 */
void MD::Motor::setSpeed(double speed) {
    if (speed < m_minSpeed && speed > m_maxSpeed*-1) { // If the speed is less than the minimum speed and greater than the maximum speed, the speed is set to the minimum speed
//    	sleep_for(nanoseconds(1000));
        m_speed = speed*-1;
    } else if (speed <= m_maxSpeed*-1) { // If the speed is less than the maximum speed, the speed is set to the maximum speed
//    	sleep_for(nanoseconds(1000));
    	m_speed = m_maxSpeed;
    } else if (speed >= m_maxSpeed) {  // If the speed is greater than the maximum speed, the speed is set to the maximum speed
//    	sleep_for(nanoseconds(1000));
        m_speed = m_maxSpeed;
    } else {                        // If the speed is greater than the minimum speed and less than the maximum speed, the speed is set to the speed
//    	sleep_for(nanoseconds(1000));
   		m_speed = speed;
    }
}

/**
 * @brief Get the Speed of the object
 * 
 * @return double 
 */
double MD::Motor::getSpeed() {
    return m_speed;
}

/**
 * @brief Get the Min Speed of the object
 * 
 * @return double 
 */
void MD::Motor::setMinSpeed(double minSpeed) {
    m_minSpeed = minSpeed;
}

// int Motor::getMinSpeed() {return 0;}

/**
 * @brief Get the Max Speed of the object
 * 
 * @return double 
 */
void MD::Motor::setMaxSpeed(double maxSpeed) {
    m_maxSpeed = maxSpeed;
}

// int Motor::getMaxSpeed() {return 0;}

/**
 * @brief Get the Min Speed of the object
 * 
 * @return double 
 */
void MD::Motor::stop() {
    m_speed = 0.0;
}
// --- End of control of motor actions ---
