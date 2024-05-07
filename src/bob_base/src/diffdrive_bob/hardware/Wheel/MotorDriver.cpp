#include "MotorDriver.hpp"

/**
 * @brief Construct a new MD::Motor::Motor object
 * 
 * @param directionPin 
 * @param pwmPin 
 * @param maxSpeed
 * @param direction
 */

namespace MD{
	
Motor::Motor(int pi_var,int directionPin,int pwmPin, double maxSpeed,int direction){
  m_DirectionPin = directionPin;
  m_PwmPin = pwmPin;
  m_MaxSpeed = int((maxSpeed/100) * MAX_SPEED);
  m_MinSpeed = MIN_SPEED;
  m_FDirection = direction;
  m_Speed = 0.0;
  m_Direction = direction;
  PWM_Range = 255;
  pi = pi_var;

  // set pins to output mode
  set_mode(pi,directionPin,PI_OUTPUT);
  set_mode(pi,pwmPin,PI_OUTPUT);

  // write forward direction to direction pin
  gpio_write(pi,m_DirectionPin, m_FDirection);
}

/**
 * @brief Gets the pin that is sending the PWM to the motor
 * 
 * @return PWM pin
 */
int Motor::getPwmPin() const{
  return m_PwmPin;
}

/**
 * @brief Gets range of PWM signal
 * 
 * @return PWM range
 */
u_int Motor::getPWMRange() const{
  return PWM_Range;
}

/**
 * @brief Sets range of PWM signal
 * 
 * @param range 
 */
void Motor::setPWMRange(uint range){
  PWM_Range= range;
}

/**
 * @brief Inverts the current direction if needed to by setSpeed and writes this to the direction pin
 * 
 */
void Motor::switchDirection() {
  m_Direction = !m_Direction;
  gpio_write(pi,m_DirectionPin, m_Direction);
}

/**
 * @brief Get the direction of the motor
 * 
 * @return String showing the driving direction of the motor and also states if the motor is idle 
 */
std::string Motor::getDirection() const{

  // check if the motor is idle
  if (m_Speed == 0){
    return "IDLE";
  }
  // check if the current direction is the same as the direction defined as forward
  if (m_Direction == m_FDirection){
    return "FORWARD";
  }
  // check if the current direction is not the same as the direction defined as forward
  else if (m_Direction != m_FDirection)
  {
    return "BACKWARD";
  }
  else{
    return "ERROR: Cannot tell which direction the motor is running";
  }
}

/**
 * @brief Returns the direction pin of the motor
 * 
 * @return Direction pin
 */
int Motor::getDirectionPin() const{
  return m_DirectionPin;
}

/** @brief Set the Speed of the motor
 *
 * @param speed
 * @details Speed is a value between -100 and 100 saying what percentage of the max speed should be sent to the motors.
 */
void Motor::setSpeed(double speed) {

  m_Speed = speed;

  // check if driving forward
  if (speed > 0){
    //check if motor direction needs to be changed
    if(m_Direction != m_FDirection){
      switchDirection();
    }
  }
  // check if driving backward
  else if (speed < 0){
    //check if motor direction needs to be changed
    if(m_Direction == m_FDirection){
      switchDirection();
    }
  }
  
  // ensure values are inbetween -100 and 100
  if (speed>m_MaxSpeed){
    m_Speed = m_MaxSpeed;
  }
  else if (speed<-m_MaxSpeed){
    m_Speed = -m_MaxSpeed;
  }

  // convert speed to percentage of max speed and map to PWM range
  double signal = fabs(m_Speed);

  set_PWM_dutycycle(pi,m_PwmPin, signal);

}

/** @brief Get the Speed of the motor
 *
 * @return speed
 */
double Motor::getSpeed() const{
  return m_Speed;
}

/** @brief Set the Max Speed of the motor
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