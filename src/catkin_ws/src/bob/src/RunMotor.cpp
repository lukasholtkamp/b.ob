#include "motor_driver_library/RunMotor.h"

#include <geometry_msgs/Twist.h> //<-- Used to receive messages from the cmd_vel topic
#include <ros/console.h> //<-- Used to change the verbosity of the program
#include <ros/ros.h> //<-- Used for the ROS environment
#include <sensor_msgs/Joy.h> //<-- Used to receive messages from the joy topic

#include "motor_driver_library/MotorDriver.h"
#include "motor_driver_library/XBoxController.h"

// motor(directionPin, PwmPin, minSpeed, maxSpeed)
MD::Motor left_motor(left_direction_pin, left_pwm_pin, kMinSpeed, kMaxSpeed); //<-- The left motor object
MD::Motor right_motor(right_direction_pin, left_pwm_pin, kMinSpeed, kMaxSpeed); //<-- The right motor object

// just for debugging
int left_motor_pwm_pin = left_motor.GetPwmPin(); //<-- The GPIO pin number for the left motor PWM
int right_motor_pwm_pin = right_motor.GetPwmPin(); //<-- The GPIO pin number for the right motor PWM

// Setup PWM for Jetson Nano   (leave in for now!)
// GPIO::PWM leftMotorPWMPin(leftMotorPwmPin, FREQUENCY); // for GPIO::BOARD
// GPIO::PWM rightMotorPWMPin(rightMotorPwmPin, FREQUENCY);


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
void MessageCallback(const geometry_msgs::Twist& cmd_vel) {
	// Getting Velocity from XboxController.cpp and calculating Speed and direction
	double linear_velocity_x = cmd_vel.linear.x; //<-- The linear velocity of the robot
	double angular_velocity_z = cmd_vel.angular.z; //<-- The angular velocity of the robot

	double left_wheel_speed = (linear_velocity_x - angular_velocity_z) * kVelocityMultiplier; //<-- The desired speed of the left wheel
	double right_wheel_speed = (linear_velocity_x + angular_velocity_z) * kVelocityMultiplier; //<-- The desired speed of the right wheel

	left_motor.SetSpeed(left_wheel_speed); //<-- Set the speed of the left motor
	right_motor.SetSpeed(right_wheel_speed); //<-- Set the speed of the right motor

	left_motor.SetDirection(left_wheel_speed, left_direction_pin); //<-- Set the direction of the left motor
  right_motor.SetDirection(-1 * right_wheel_speed, right_direction_pin); //<-- Set the direction of the right motor

  bool left_wheel_direction = left_motor.GetDirection();
  bool right_wheel_direction = right_motor.GetDirection(); // reversed to mirror the left motor

	// Speed after putting it on Range of 0-100
	double new_left_wheel_speed = left_motor.GetSpeed(); //<-- The new speed of the left wheel
	double new_right_wheel_speed = right_motor.GetSpeed(); //<-- The new speed of the right wheel

	// Writing speed to Pwm Pin to Control the Speed
	// PWM takes speed to set power of Motor in %
	//leftMotorPWMPin.ChangeDutyCycle(newLeftWheelSpeed); //<-- Set the speed of the left motor
	//rightMotorPWMPin.ChangeDutyCycle(newRightWheelSpeed); //<-- Set the speed of the right motor
	softPwmWrite(left_pwm_pin, new_left_wheel_speed); //<-- Set the speed of the left motor
	softPwmWrite(right_pwm_pin, new_right_wheel_speed); //<-- Set the speed of the right motor

	ROS_INFO_STREAM("------------------------------------");
	ROS_INFO_STREAM("Linear velocity: " << linear_velocity_x);
	ROS_INFO_STREAM("Angular velocity: " << angular_velocity_z);

	ROS_INFO_STREAM("Left wheel speed: " << left_wheel_speed);
	ROS_INFO_STREAM("New left wheelSpeed: " << new_left_wheel_speed);

	ROS_INFO_STREAM("Right wheel speed: " << right_wheel_speed);
	ROS_INFO_STREAM("New right wheelSpeed: " << new_right_wheel_speed);

	ROS_INFO_STREAM("Left Wheel Direction: " << left_wheel_direction);
	ROS_INFO_STREAM("Right Wheel Direction: " << right_wheel_direction);

	if (linear_velocity_x > 0 && angular_velocity_z < 0) {
		ROS_INFO_STREAM("Forward right");
	} else if (linear_velocity_x > 0 && angular_velocity_z > 0) {
		ROS_INFO_STREAM("Forward left");
	} else if (linear_velocity_x < 0 && angular_velocity_z < 0) {
		ROS_INFO_STREAM("Backwards right");
	} else if (linear_velocity_x < 0 && angular_velocity_z > 0) {
		ROS_INFO_STREAM("Backwards left");
	} else if (linear_velocity_x == 0 && angular_velocity_z > 0) {
		ROS_INFO_STREAM("Left");
	} else if (linear_velocity_x > 0 && angular_velocity_z == 0) {
		ROS_INFO_STREAM("Forward");
	} else if (linear_velocity_x < 0 && angular_velocity_z == 0) {
		ROS_INFO_STREAM("Backwards");
	} else if (linear_velocity_x == 0 && angular_velocity_z < 0) {
		ROS_INFO_STREAM("Right");
	} else {
		left_motor.Stop();
		right_motor.Stop();
		ROS_INFO_STREAM("Not moving");
	}
	ROS_INFO_STREAM("------------------------------------");

}

/**
 * @brief Main function
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

	ros::Subscriber sub = nh.subscribe("cmd_vel", kFrequency, &MessageCallback);	

	ros::spin();
	ROS_INFO_STREAM("spin");
	// Equivalent of JetsonGpio  GPIO.Cleanup();
	softPwmWrite(left_pwm_pin,0);
	digitalWrite(left_pwm_pin,LOW);
	softPwmWrite(right_pwm_pin,0);
	digitalWrite(right_pwm_pin,LOW);
	digitalWrite(right_direction_pin,LOW);
	digitalWrite(right_direction_pin,LOW);

	ROS_INFO_STREAM("clean");
}
