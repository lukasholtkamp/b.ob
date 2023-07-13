#include "MotorDriver.h"
#include "RunMotor.h"

/**
 * @brief Construct a new MD::Motor::Motor object
 * 
 * @param directionPin 
 * @param pwmPin 
 * @param minSpeed 
 * @param maxSpeed 
 */
MD::Motor::Motor(int directionPin,int pwmPin, double minSpeed, double maxSpeed) {
	m_DirectionPin = directionPin;
	m_PwmPin = pwmPin;
	m_MinSpeed = minSpeed;
	m_MaxSpeed = maxSpeed;
	m_Speed = 0.0;
	m_Direction = 0;
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

/// @brief Returns the pwm pin of the motor
/// @return int 
int MD::Motor::getPwmPin(){
	return m_PwmPin;
}

/**
 *
 *
 *
 *
 */

double MD::Motor::LinearAndAngularVelocities(double linearVelocityX, double angularVelocityZ) {

	double speed;
    	/* if(linearVelocityX > 0 && angularVelocityZ < 0 && ( (angularVelocityZ*-1) <= linearVelocityX) ){ //this will slow down one wheel an speed up the other wheel  
    	 	speed = (linearVelocityX - angularVelocityZ); 
       		return speed;
	 }*/
        if(linearVelocityX > 0 && angularVelocityZ < 0 ){ //this will only affect the wheel that has to be slowed down
                speed = linearVelocityX;
		return speed; 
        }
        else if(linearVelocityX > 0 && angularVelocityZ > 0 && (angularVelocityZ <= linearVelocityX) ){
                speed  = (linearVelocityX - angularVelocityZ);
                return speed;
        }
        else if(linearVelocityX > 0 &&  angularVelocityZ > 0 && (angularVelocityZ > linearVelocityX) ) {
                speed = 0;
                return speed;
        }


        /*else if(linearVelocityX < 0 && angularVelocityZ < 0 && ( (angularVelocityZ*-1) <= (linearVelocityX*-1)  ){
                speed = (linearVelocityX + angularVelocityZ); 
                return speed;
        }*/
	else if (linearVelocityX < 0 && angularVelocityZ < 0) {
		speed = linearVelocityX;
		return speed;
	}
        else if(linearVelocityX < 0 &&  angularVelocityZ < 0 && ( (angularVelocityZ *-1) > (linearVelocityX*-1) ) ){
                speed = 0;
                return speed;
        }
        else if(linearVelocityX < 0 && angularVelocityZ > 0 && (angularVelocityZ <= (linearVelocityX*-1) ) ){
                speed = (linearVelocityX + angularVelocityZ);
                return speed;
        }
        else if(linearVelocityX < 0 && angularVelocityZ > 0 && (angularVelocityZ > (linearVelocityX*-1) ) ) {
                speed = 0;
                return speed;
        }
        else if(linearVelocityX == 0 ){
        	speed = (linearVelocityX - angularVelocityZ) ;
        	return speed;
        }

}


/**
 * @brief Set the direction of the object
 * 

 * @param speed 
 * @param pinNumber 
 * @details This function is used to set the direction of the motor. If the speed is positive, the direction is forward. If the speed is negative, the direction is backward. */
void MD::Motor::setDirection(double speed, int directionPin) {
	if (speed > m_MinSpeed) {
		m_Direction = true;
		digitalWrite(directionPin, HIGH);
	} else if (speed < m_MinSpeed) {
		m_Direction = false;
		digitalWrite(directionPin, LOW);
	}
}

/**
 * @brief Get the direction of the object
 * 
 * @return true 
 * @return false 
 */
bool MD::Motor::getDirection() {
	return m_Direction;
}

/// @brief Returns the direction pin of the motor
int MD::Motor::getDirectionPin(){
	return m_DirectionPin;
}

/**
 * @brief Set the Speed of the object
 * 
 * @param speed 
 */
void MD::Motor::setSpeed(double speed) {
    if (speed < m_MinSpeed && speed > m_MaxSpeed*-1) { // If the speed is less than the minimum speed and greater than the maximum speed, the speed is set to the minimum speed
        m_Speed = speed*-1;
    } else if (speed <= m_MaxSpeed*-1) { // If the speed is less than the maximum speed, the speed is set to the maximum speed
    	m_Speed = m_MaxSpeed;
    } else if (speed >= m_MaxSpeed) {  // If the speed is greater than the maximum speed, the speed is set to the maximum speed
        m_Speed = m_MaxSpeed;
    } else {                        // If the speed is greater than the minimum speed and less than the maximum speed, the speed is set to the speed
   		m_Speed = speed;
    }
}

/**
 * @brief Get the Speed of the object
 * 
 * @return double 
 */
double MD::Motor::getSpeed() {
	return m_Speed;
}

/**
 * @brief Set the Min Speed of the object
 * 
 * @return double 
 */
void MD::Motor::setMinSpeed(double minSpeed) {
	m_MinSpeed = minSpeed;
}

/**
 * @brief Set the Max Speed of the object
 * 
 * @return double 
 */
void MD::Motor::setMaxSpeed(double maxSpeed) {
	m_MaxSpeed = maxSpeed;
}

/**
 * @brief Stop the motor
 * 
 */
void MD::Motor::stop() {
	m_Speed = 0.0;
}
