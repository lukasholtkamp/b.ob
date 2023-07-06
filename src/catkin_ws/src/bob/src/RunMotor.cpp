#include "MotorDriver.h" //<-- Used for the Motor class
#include <ros/ros.h> //<-- Used for the ROS environment
#include <geometry_msgs/Twist.h> //<-- Used to receive messages from the cmd_vel topic
#include <sensor_msgs/Joy.h> //<-- Used to receive messages from the joy topic
#include <ros/console.h> //<-- Used to change the verbosity of the program
//#include <JetsonGPIO.h> //<-- Used to control the GPIO pins on the Jetson
#include <wiringPi.h> //<-- Used to control the GPIO pins on the Raspberry Pi
#include <softPwm.h> //<-- Used to control the PWM pins on the Raspberry Pi
#include <chrono> //<-- Used to measure time
#include <thread> //<-- Used to make the program sleep

using namespace std::this_thread; //<-- Used to make the program sleep
using namespace std::chrono; //<-- Used to make the program sleep

#define LEFT_DIRECTION_PIN 	22 //<-- The GPIO pin number for the left motor direction
#define LEFT_PWM_PIN 		26 //<-- The GPIO pin number for the left motor PWM
#define RIGHT_DIRECTION_PIN 24 //<-- The GPIO pin number for the right motor direction
#define RIGHT_PWM_PIN		23 //<-- The GPIO pin number for the right motor PWM

// motor(directionPin, PwmPin, minSpeed, maxSpeed)
MD::Motor leftMotor(LEFT_DIRECTION_PIN, LEFT_PWM_PIN, 0.0, 100.0); //<-- The left motor object
MD::Motor rightMotor(RIGHT_DIRECTION_PIN, RIGHT_PWM_PIN, 0.0, 100.0); //<-- The right motor object

int leftMotorPwmPin = leftMotor.getPwmPin(); //<-- The GPIO pin number for the left motor PWM
int rightMotorPwmPin = rightMotor.getPwmPin(); //<-- The GPIO pin number for the right motor PWM

int leftMotorDirection = leftMotor.getDirection(); //<-- The GPIO pin number for the left motor direction
int rightMotorDirection = rightMotor.getDirection(); //<-- The GPIO pin number for the right motor direction

//GPIO::PWM leftMotorPWMPin(leftMotorPwmPin, 1000); // for GPIO::BOARD
//GPIO::PWM rightMotorPWMPin(rightMotorPwmPin, 1000);

double leftWheelSpeed = leftMotor.getSpeed(); //<-- The speed of the left wheel
double rightWheelSpeed = rightMotor.getSpeed(); //<-- The speed of the right wheel

/** @brief Callback function for the subscriber
* @param cmd_vel 
* @returns void
* @details This function is called when a message is received on the cmd_vel topic
* The message is a geometry_msgs::Twist message, which contains the linear and angular velocities
* The linear velocity is the desired velocity of the robot, and the angular velocity is the desired
* angular velocity of the robot. These values are used to calculate the desired speed of the left and
* right wheels of the robot. The speed of the wheels is given in per cent, and is calculated by
* linearVelocityX - angularVelocityZ * 100.0 for the left wheel and linearVelocityX + angularVelocityZ * 100.0
* for the right wheel. The speed is then used to set the speed of the left and right motors. The direction of
* the wheels is also set using the speed. If the speed is positive, the direction is set to forward, and if the
* speed is negative, the direction is set to backward. The speed is then set to the PWM pin of the motor, and the
* direction is set to the direction pin of the motor. The speed is also set to the ROS_INFO_STREAM, which is used
* by the ROS_INFO_STREAM to print the speed to the console.
*/
void messageCallback(const geometry_msgs::Twist& cmd_vel) {

	double linearVelocityX = cmd_vel.linear.x; //<-- The linear velocity of the robot
	double angularVelocityZ = cmd_vel.angular.z; //<-- The angular velocity of the robot

	leftWheelSpeed = (linearVelocityX - angularVelocityZ) * 100.0; //<-- The desired speed of the left wheel
	rightWheelSpeed = (linearVelocityX + angularVelocityZ) * 100.0; //<-- The desired speed of the right wheel

	leftMotor.setSpeed(leftWheelSpeed); //<-- Set the speed of the left motor
	rightMotor.setSpeed(rightWheelSpeed); //<-- Set the speed of the right motor

	double newLeftWheelSpeed = leftMotor.getSpeed(); //<-- The new speed of the left wheel
	double newRightWheelSpeed = rightMotor.getSpeed(); //<-- The new speed of the right wheel

	leftMotor.setDirection(leftWheelSpeed, LEFT_DIRECTION_PIN); //<-- Set the direction of the left motor
	rightMotor.setDirection(-1*rightWheelSpeed, RIGHT_DIRECTION_PIN); //<-- Set the direction of the right motor


	bool LeftWheelDirection = leftMotor.getDirection(); //<-- The direction of the left wheel
	bool RightWheelDirection = rightMotor.getDirection(); //<-- Reversed to mirror the left motor


	//leftMotorPWMPin.ChangeDutyCycle(newLeftWheelSpeed); //<-- Set the speed of the left motor
	//rightMotorPWMPin.ChangeDutyCycle(newRightWheelSpeed); //<-- Set the speed of the right motor
	softPwmWrite(LEFT_PWM_PIN,newLeftWheelSpeed); //<-- Set the speed of the left motor
	softPwmWrite(RIGHT_PWM_PIN,newRightWheelSpeed); //<-- Set the speed of the right motor

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
	//sleep_for(nanoseconds(10000));

}

/**
 * @brief Main function
 * 
 * @param argc 
 * @param argv 
 * @return int 
 * @details This function is the main function of the program. It starts the left and right motor PWM pins
 * and sets the speed of the motors to 0.0. It then starts the ROS node, and subscribes to the cmd_vel topic.
 * It then spins the node, which means that it waits for messages to be received on the cmd_vel topic. When a
 * message is received, the messageCallback function is called.
 */
int main(int argc, char** argv) {
//	leftMotorPWMPin.start(0.0);
//	rightMotorPWMPin.start(0.0);

	ros::init(argc, argv, "run_motor");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("cmd_vel", 1000, &messageCallback);	

	ROS_INFO_STREAM("Left pwm pin: " << leftMotorPwmPin);
	ROS_INFO_STREAM("Right pwm pin: " << rightMotorPwmPin);

	ros::spin();
	ROS_INFO_STREAM("spin");
	/*softPwmWrite(LEFT_PWM_PIN,0);
	digitalWrite(LEFT_PWM_PIN,LOW);
	softPwmWrite(RIGHT_PWM_PIN,0);
	digitalWrite(RIGHT_PWM_PIN,LOW);
	digitalWrite(RIGHT_DIRECTION_PIN,LOW);
	digitalWrite(LEFT_DIRECTION_PIN,LOW);*/
	ROS_INFO_STREAM("clean");
}
