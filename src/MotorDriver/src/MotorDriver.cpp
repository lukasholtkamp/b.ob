#include "/home/bertrandt/BOB/library/MotorDriver/include/MotorDriver.h"

MD::Motor::Motor(int directionPin, int pwmPin, int minSpeed, int maxSpeed) {
    m_directionPin = directionPin;
    m_pwmPin = pwmPin;
    m_minSpeed = minSpeed;
    m_maxSpeed = maxSpeed;
    m_speed = 0;
    m_direction = true;

    // Pin configurations for JetsonGPIO
    GPIO::setmode(GPIO::BOARD); // GPIO::BCM or GPIO::BOARD
    GPIO::setup(m_directionPin, GPIO::OUT, GPIO::HIGH);
    GPIO::setup(m_pwmPin, GPIO::OUT, GPIO::HIGH);
}

// --- Pwm pin ---
int MD::Motor::getPwmPin(){
    return m_pwmPin;
}
// --- End of Pwm pin ---

// --- Wheel direction ---
void MD::Motor::setDirection(int16_t speed) {
    m_direction = speed;
    if (speed >= 0) {
        GPIO::output(m_directionPin, GPIO::HIGH);
    } else {
        GPIO::output(m_directionPin, GPIO::LOW);
    }
}

// bool Motor::getDirection() {return true;}

int MD::Motor::getDirectionPin(){
    return m_directionPin;
}
// --- End of wheel direction ---

// --- Control of motor actions ---
void MD::Motor::setSpeed(int16_t speed) {
    if (speed < m_minSpeed) {
        speed = m_minSpeed;
    } else if (speed > m_maxSpeed) {
        speed = m_maxSpeed;
    }

    m_speed = speed;
}

int16_t MD::Motor::getSpeed() {
    return m_speed;
}

void MD::Motor::setMinSpeed(int minSpeed) {
    m_minSpeed = minSpeed;
}

// int Motor::getMinSpeed() {return 0;}

void MD::Motor::setMaxSpeed(int maxSpeed) {
    m_maxSpeed = maxSpeed;
}

// int Motor::getMaxSpeed() {return 0;}

/*
void MD::Motor::startMotor(int speed) {
    if (speed < m_minSpeed) {
        speed = m_minSpeed;
    } else if (speed > m_maxSpeed) {
        speed = m_maxSpeed;
    }

    if (m_direction) {
        GPIO::output(m_directionPin, GPIO::HIGH);
    } else {
        GPIO::output(m_directionPin, GPIO::LOW);
    }
}
*/

void MD::Motor::stop() {
    m_speed = 0;
}
// --- End of control of motor actions ---

