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
	
Motor::Motor(int directionPin,int pwmPin, double minSpeed, double maxSpeed,int direction){
  m_DirectionPin = directionPin;
  m_PwmPin = pwmPin;
  m_MinSpeed = minSpeed;
  m_MaxSpeed = maxSpeed;
  m_FDirection = direction;
  m_Speed = 0.0;
  m_Direction = IDLE;
  PWM_Range = 255;

  
  gpioSetMode(directionPin,PI_OUTPUT);
  gpioSetMode(pwmPin,PI_OUTPUT);
  
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

double Motor::LinearAndAngularVelocities(double linearVelocityX, double angularVelocityZ) {

  double speed;
  //---Forward Driving Curve---
  /* if(linearVelocityX > 0 && angularVelocityZ < 0 && ( (angularVelocityZ*-1) <= linearVelocityX) ){ //this will slow down one wheel an speed up the other wheel
    speed = (linearVelocityX - angularVelocityZ);
    return speed;
  }*/
  if(linearVelocityX > 0 && angularVelocityZ < 0){ //this will only influence the wheel that has to be slowed down 
    speed = linearVelocityX;
    return speed;
  }
  else if(linearVelocityX > 0 && angularVelocityZ > 0 ){
    speed = (linearVelocityX - angularVelocityZ);
    return speed;
  }
  //---Backwards Driving Curve---
  else if (linearVelocityX < 0 && angularVelocityZ < 0) {
    speed = linearVelocityX;
    return speed;
  }
  /*else if(linearVelocityX < 0 && angularVelocityZ < 0 && ( (angularVelocityZ*-1) <= (linearVelocityX*-1) ) ){
    speed = (linearVelocityX + angularVelocityZ);
    return speed;
  }*/
  else if(linearVelocityX < 0 && angularVelocityZ > 0 && (angularVelocityZ <= (linearVelocityX*-1) ) ){
    speed = (linearVelocityX + angularVelocityZ);
    return speed;
  }
  else if(linearVelocityX < 0 && angularVelocityZ > 0 && (angularVelocityZ > (linearVelocityX*-1) ) ) {
    speed = 0;
    return speed;
  }
  //---Rotating---
  else if(linearVelocityX == 0 ){
    speed = (linearVelocityX - angularVelocityZ);
    return speed;
  }
  else {
	speed = 0;
	return speed;  
  }

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

  if (m_Direction == m_FDirection){
    return "FORWARD";
  }
  else if (m_Direction != m_FDirection)
  {
    return "BACKWARD";
  }
  else if (m_Direction == IDLE){
    return "IDLE";
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

  if (speed > m_MinSpeed){
    if(m_Direction != m_FDirection){
      // direction change
      gpioPWM(m_PwmPin, 1);
      switchDirection();
    }
  }
  else if (speed < m_MinSpeed){
    if(m_Direction == m_FDirection){
      // direction change
      gpioPWM(m_PwmPin, 1);
      switchDirection();
    }
  }
  else if (speed == m_MinSpeed){
    gpioPWM(m_PwmPin, 1);
    m_Direction = IDLE;
  }
  else if (speed>m_MaxSpeed){
    m_Speed = m_MaxSpeed;
  }

  m_Speed = speed;

  auto intValue = static_cast<uint8_t>( PWM_Range * abs(m_Speed/m_MaxSpeed));
  gpioPWM(m_PwmPin, intValue);

}

/** @brief Get the Speed of the object
 *
 * @return double
 */
double Motor::getSpeed() const{
  return m_Speed;
}

/** @brief Set the Min Speed of the object
 *
 * @return double
 */
void Motor::setMinSpeed(double minSpeed) {
  m_MinSpeed = minSpeed;
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
