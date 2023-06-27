#include "MotorDriver.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <ros/console.h>
#include <JetsonGPIO.h>

#define LEFT_DIRECTION_PIN 	31
#define LEFT_PWM_PIN 		32
#define RIGHT_DIRECTION_PIN 	35
#define RIGHT_PWM_PIN		33

// motor(directionPin, PwmPin, minSpeed, maxSpeed)
MD::Motor leftMotor(LEFT_DIRECTION_PIN, LEFT_PWM_PIN, 0.0, 100.0);
MD::Motor rightMotor(RIGHT_DIRECTION_PIN, RIGHT_PWM_PIN, 0.0, 100.0);

 int leftMotorPwmPin = leftMotor.getPwmPin();
 int rightMotorPwmPin = rightMotor.getPwmPin();

int leftMotorDirection = leftMotor.getDirection();
int rightMotorDirection = rightMotor.getDirection();

GPIO::PWM leftMotorPWMPin(leftMotorPwmPin, 1000); // for GPIO::BOARD
GPIO::PWM rightMotorPWMPin(rightMotorPwmPin, 1000);

double leftWheelSpeed = leftMotor.getSpeed();
double rightWheelSpeed = rightMotor.getSpeed();


void messageCallback(const geometry_msgs::Twist& cmd_vel) {

	double linearVelocityX = cmd_vel.linear.x;
	double angularVelocityZ = cmd_vel.angular.z;

	leftWheelSpeed = (linearVelocityX - angularVelocityZ) * 100.0;
	rightWheelSpeed = (linearVelocityX + angularVelocityZ) * 100.0;

	leftMotor.setSpeed(leftWheelSpeed);
	rightMotor.setSpeed(rightWheelSpeed);

	double newLeftWheelSpeed = leftMotor.getSpeed()/10;
	double newRightWheelSpeed = rightMotor.getSpeed()/10;
	

	leftMotor.setDirection(leftWheelSpeed, LEFT_DIRECTION_PIN);
	rightMotor.setDirection(rightWheelSpeed, RIGHT_DIRECTION_PIN);

	
	bool LeftWheelDirection = leftMotor.getDirection();
	bool RightWheelDirection = !rightMotor.getDirection(); // reversed to mirror the left motor
	

	leftMotorPWMPin.ChangeDutyCycle(newLeftWheelSpeed);
	rightMotorPWMPin.ChangeDutyCycle(newRightWheelSpeed);
	

	ROS_INFO_STREAM("------------------------------------");
	ROS_INFO_STREAM("Linear velocity: " << linearVelocityX);
	ROS_INFO_STREAM("Angular velocity: " << angularVelocityZ);

	ROS_INFO_STREAM("Left wheel speed: " << leftWheelSpeed);
	ROS_INFO_STREAM("New left wheelSpeed: " << newLeftWheelSpeed);

	ROS_INFO_STREAM("Right wheel speed: " << rightWheelSpeed);
	ROS_INFO_STREAM("New right wheelSpeed: " << newRightWheelSpeed);
	
	ROS_INFO_STREAM("Left Wheel Direction: " << LeftWheelDirection);
	ROS_INFO_STREAM("Right Wheel Direction: " << RightWheelDirection);

	if(linearVelocityX > 0 && angularVelocityZ < 0) {
		ROS_INFO_STREAM("Forward right");
	} else if(linearVelocityX > 0 && angularVelocityZ > 0) {
		ROS_INFO_STREAM("Forward left");
	} else if(linearVelocityX < 0 && angularVelocityZ < 0) {
		ROS_INFO_STREAM("Backwards right");
	} else if(linearVelocityX < 0 && angularVelocityZ > 0) {
		ROS_INFO_STREAM("Backwards left");
	} else if(linearVelocityX == 0 && angularVelocityZ > 0) {
		ROS_INFO_STREAM("Left");
	} else if(linearVelocityX > 0 && angularVelocityZ == 0) {
		ROS_INFO_STREAM("Forward");
	} else if(linearVelocityX < 0 && angularVelocityZ == 0) {
		ROS_INFO_STREAM("Backwards");
	} else if(linearVelocityX == 0 && angularVelocityZ < 0) {
		ROS_INFO_STREAM("Right");
	} else {
		leftMotor.stop();
		rightMotor.stop();
		ROS_INFO_STREAM("Not moving");
	}
	ROS_INFO_STREAM("------------------------------------");
}

int main(int argc, char** argv) {
	leftMotorPWMPin.start(0.0);
	rightMotorPWMPin.start(0.0);
	
	ros::init(argc, argv, "run_motor");
	ros::NodeHandle nh;
		
	ros::Subscriber sub = nh.subscribe("cmd_vel", 1000, &messageCallback);	
	
	ROS_INFO_STREAM("Left pwm pin: " << leftMotorPwmPin);
	ROS_INFO_STREAM("Right pwm pin: " << rightMotorPwmPin);
	
	ros::spin();
	ROS_INFO_STREAM("spin");
	GPIO::cleanup();
	ROS_INFO_STREAM("clean");
	
}
