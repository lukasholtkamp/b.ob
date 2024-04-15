//~ # GNU nano 4.8                                                                                                                                                                                                                                                                                                                                                                                                                                                       MotorDriver.cpp                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
#include "MotorDriver.hpp"
#include "RunMotor.hpp"

/**
 * @brief Construct a new MD::Motor::Motor object
 * 
 * @param directionPin 
 * @param pwmPin 
 * @param minSpeed 
 * @param maxSpeed 
 */

namespace MD{
	
Motor::Motor(int directionPin,int pwmPin, double maxSpeed,int direction){
  m_DirectionPin = directionPin;
  m_PwmPin = pwmPin;
  m_MaxSpeed = maxSpeed;
  m_FDirection = direction;
  m_Speed = 0.0;
  m_Direction = direction;
  PWM_Range = 255;

  
  gpioSetMode(directionPin,PI_OUTPUT);
  gpioSetMode(pwmPin,PI_OUTPUT);

  gpioWrite(m_DirectionPin, m_FDirection);
}


/// @brief Returns the pwm pin of the motor
/// @return int
int Motor::getPwmPin() const{
  return m_PwmPin;
}

u_int Motor::getPWMRange() const{
  return PWM_Range;
}

void Motor::setPWMRange(uint range){
  PWM_Range= range;
}

/** @brief Set the direction of the object
 *

 * @param speed
 * @param pinNumber
 * @details This function is used to set the direction of the motor. If the speed is positive, the direction is forward. If the speed is negative, the direction is backward. */
void Motor::switchDirection() {
  m_Direction = !m_Direction;
  gpioWrite(m_DirectionPin, m_Direction);
}

/**
 * @brief Get the direction of the object
 *
 * @return true
 * @return false
 */
std::string Motor::getDirection() const{

  if (m_Speed == 0){
    return "IDLE";
  }
  if (m_Direction == m_FDirection){
    return "FORWARD";
  }
  else if (m_Direction != m_FDirection)
  {
    return "BACKWARD";
  }
}

/// @brief Returns the direction pin of the motor
int Motor::getDirectionPin() const{
  return m_DirectionPin;
}

/** @brief Set the Speed of the object
 *
 * @param speed
 */
void Motor::setSpeed(double speed) {

  m_Speed = speed;

  if (speed > 0){
    if(m_Direction != m_FDirection){
      // direction change
      switchDirection();
    }
  }
  else if (speed < 0){
    if(m_Direction == m_FDirection){
      // direction change
      switchDirection();
    }
  }
  
  if (speed>100){
    m_Speed = 100;
  }
  else if (speed<-100){
    m_Speed = -100;
  }

  double signal = (PWM_Range/m_MaxSpeed) * ((m_MaxSpeed/100) * abs(speed));

  gpioPWM(m_PwmPin, signal);

}

/** @brief Get the Speed of the object
 *
 * @return double
 */
double Motor::getSpeed() const{
  return m_Speed;
}

/** @brief Set the Max Speed of the object
 *
 * @return double
 */
void Motor::setMaxSpeed(double maxSpeed) {
  m_MaxSpeed = maxSpeed;
}

/**
 * @brief Stop the motor
 *
 */
void Motor::stop() {
  m_Speed = 0.0;
  setSpeed(m_Speed);
}

}
