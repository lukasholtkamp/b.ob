#include "MotorDriver.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
//#include <JetsonGPIO.h>


// motor(directionPin, PwmPin, minSpeed, maxSpeed)
MD::Motor leftMotor(31, 32, -255.0, 255.0);
//MD::Motor rightMotor(35, 33, -255, 255);

int leftMotorPwmPin = leftMotor.getPwmPin();
//int rightMotorPwmPin = rightMotor.getPwmPin();

float linearVelocityX;
float angularVelocityZ;

//ros::NodeHandle nh;
//geometry_msgs::Twist msg;

void messageCallback(const geometry_msgs::Twist& cmd_vel) {
	GPIO::PWM leftMotorPWMPin(leftMotorPwmPin, 1000); // for GPIO::BOARD
	//GPIO::PWM rightMotorPWMPin(rightMotorPwmPin, 100);
	
	leftMotorPWMPin.start(0.0);
	//rightMotorPWMPin.start(10.0);

	linearVelocityX = cmd_vel.linear.x;
	angularVelocityZ = cmd_vel.angular.z;
	
	float leftWheel = (linearVelocityX - angularVelocityZ) * 100;
	//int rightWheel = (linearVelocityX + angularVelocityZ) * 100;
	
	leftMotor.setSpeed(leftWheel);
	//rightMotor.setSpeed(rightWheel);
	
	float newLeftWheel = leftMotor.getSpeed();
	//int newRightWheel = rightMotor.getSpeed();
	
	leftMotorPWMPin.ChangeDutyCycle(newLeftWheel);
	//rightMotorPWMPin.ChangeDutyCycle(newRightWheel);
	
	if (newLeftWheel == 0.0) {
		leftMotor.stop();
		//rightMotor.stop();
		
		leftMotorPWMPin.stop();
		//rightMotorPWMPin.stop();
		
		
	}
}

float linx, angZ;

int main(int argc, char** argv) {
	
	ros::init(argc, argv, "run_motor");
	ros::NodeHandle nh;
	
	geometry_msgs::Twist msg;
	msg.linear.x = linx;     
	msg.angular.z = angZ;
	//ros::Subscriber <geometry_msgs::Twist> sub("/cmd_vel", &messageCallback);
	ros::Subscriber sub = nh.subscribe("/cmd_vel", 10, &messageCallback);
	
	ROS_INFO_STREAM("Subscriber velocities:"<<" linear="<<linx<<" angular="<<angZ);
	
	ros::spin();
	GPIO::cleanup();
}
