#include "/home/bertrandt/BOB/library/MotorDriver/include/MotorDriver.h"

int main() {
	float linearVelocity = 0.0;
	float angularVelocity = 0.0;

    // motor(directionPin, PwmPin, minSpeed, maxSpeed)
    MD::Motor leftMotor(31, 32, -255, 255);
    MD::Motor rightMotor(33, 34, -255, 255);
    
    int leftMotorPwmPin = leftMotor.getPwmPin();
    int rightMotorPwmPin = rightMotor.getPwmPin();

    GPIO::PWM leftMotorPWMPin(leftMotorPwmPin, 1000); // for GPIO::BOARD
    GPIO::PWM rightMotorPWMPin(rightMotorPwmPin, 1000);
    
    int leftSpeed = (linearVelocity + angularVelocity) * 100;
    int rightSpeed = (linearVelocity - angularVelocity) * 100;
    
    leftMotorPWMPin.start(0.0);
    rightMotorPWMPin.start(0.0);
    
    leftMotor.setSpeed(leftSpeed);
    int newLeftSpeed = leftMotor.getSpeed();
    
    rightMotor.setSpeed(rightSpeed);
    int newRightSpeed = rightMotor.getSpeed();
    
    leftMotorPWMPin.ChangeDutyCycle(newLeftSpeed);
    rightMotorPWMPin.ChangeDutyCycle(newRightSpeed);
    
    leftMotor.stop();
    rightMotor.stop();
    
    leftMotorPWMPin.stop();
    rightMotorPWMPin.stop();

    GPIO::cleanup();

    return 0;
}

