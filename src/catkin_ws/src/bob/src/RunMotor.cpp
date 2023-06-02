#include "MotorDriver.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>

// motor(directionPin, PwmPin, minSpeed, maxSpeed)
MD::Motor leftMotor(31, 32, 0.0, 100.0);
MD::Motor rightMotor(35, 33, 0.0, 100.0);

int leftMotorPwmPin = 32; // leftMotor.getPwmPin();
int rightMotorPwmPin = 33 // rightMotor.getPwmPin();

int leftMotorDirection = leftMotor.getDirection();
int rightMotorDirection = rightMotor.getDirection();

GPIO::PWM leftMotorPWMPin(32, 1000); // for GPIO::BOARD
GPIO::PWM rightMotorPWMPin(33, 1000);

double leftWheel = leftMotor.getSpeed();
double rightWheel = rightMotor.getSpeed();

void messageCallback(const geometry_msgs::Twist& cmd_vel) {
	
	double linearVelocityX = cmd_vel.linear.x;
	double angularVelocityZ = cmd_vel.angular.z;
	
	leftWheel = (linearVelocityX - angularVelocityZ) * 100.0;
	rightWheel = (linearVelocityX + angularVelocityZ) * 100.0;
	
	leftMotor.setSpeed(leftWheel);
	rightMotor.setSpeed(rightWheel);
	
	double newLeftWheel = leftMotor.getSpeed();
	double newRightWheel = rightMotor.getSpeed();
	
	
	
	leftMotorPWMPin.ChangeDutyCycle(newLeftWheel);
	rightMotorPWMPin.ChangeDutyCycle(newRightWheel);
	
	ROS_INFO_STREAM("------------------------------------");
	ROS_INFO_STREAM("Linear velocity: " << linearVelocityX);
	ROS_INFO_STREAM("Angular velocity: " << angularVelocityZ);
	ROS_INFO_STREAM("Left wheel speed: " << leftWheel);
	ROS_INFO_STREAM("New left wheelSpeed: " << newLeftWheel);
	ROS_INFO_STREAM("Right wheel speed: " << rightWheel);
	ROS_INFO_STREAM("New right wheelSpeed: " << newRightWheel);
	
    if(linearVelocityX > 0 && angularVelocityZ < 0) {
		ROS_INFO_STREAM("Forward right");
	} else if(linearVelocityX > 0 && angularVelocityZ > 0) {
		ROS_INFO_STREAM("Forward left");
	} else if(linearVelocityX < 0 && angularVelocityZ < 0) {
		ROS_INFO_STREAM("Backwards right");
	} else if(linearVelocityX < 0 && angularVelocityZ > 0) {
		ROS_INFO_STREAM("Backwards left");
	} else if(linearVelocityX == 0 && angularVelocityZ == 1) {
		ROS_INFO_STREAM("Left");
	} else if(linearVelocityX == 1 && angularVelocityZ == 0) {
		ROS_INFO_STREAM("Forward");
	} else if(linearVelocityX == -1 && angularVelocityZ == 0) {
		ROS_INFO_STREAM("Backwards");
	} else if(linearVelocityX == 0 && angularVelocityZ == -1) {
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
}
