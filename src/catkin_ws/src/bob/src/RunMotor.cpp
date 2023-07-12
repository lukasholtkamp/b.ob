#include "MotorDriver.h"
#include "RunMotor.h"

// motor(directionPin, PwmPin, minSpeed, maxSpeed)
MD::Motor leftMotor(LEFT_DIRECTION_PIN, LEFT_PWM_PIN, MIN_SPEED, MAX_SPEED); //<-- The left motor object
MD::Motor rightMotor(RIGHT_DIRECTION_PIN, RIGHT_PWM_PIN, MIN_SPEED, MAX_SPEED); //<-- The right motor object

// just for debugging
int leftMotorPwmPin = leftMotor.getPwmPin(); //<-- The GPIO pin number for the left motor PWM
int rightMotorPwmPin = rightMotor.getPwmPin(); //<-- The GPIO pin number for the right motor PWM

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
void messageCallback(const geometry_msgs::Twist& cmd_vel) {
	// Getting Velocity from XboxController.cpp and calculating Speed and direction
	double linearVelocityX = cmd_vel.linear.x; //<-- The linear velocity of the robot
	double angularVelocityZ = cmd_vel.angular.z; //<-- The angular velocity of the robot

	//double leftWheelSpeed = (linearVelocityX - angularVelocityZ) * VELOCITY_MULTIPLIER; //<-- The desired speed of the left wheel
	//double rightWheelSpeed = (linearVelocityX + angularVelocityZ) * VELOCITY_MULTIPLIER; //<-- The desired speed of the right wheel

	double leftWheelSpeed = leftMotor.LinearAndAngularVelocities(linearVelocityX, angularVelocityZ);
	double rightWheelSpeed = rightMotor.LinearAndAngularVelocities(linearVelocityX, angularVelocityZ);


	leftMotor.setSpeed(leftWheelSpeed); //<-- Set the speed of the left motor
	rightMotor.setSpeed(rightWheelSpeed); //<-- Set the speed of the right motor

	leftMotor.setDirection(leftWheelSpeed, LEFT_DIRECTION_PIN); //<-- Set the direction of the left motor
	rightMotor.setDirection(-1*rightWheelSpeed, RIGHT_DIRECTION_PIN); //<-- Set the direction of the right motor

	bool LeftWheelDirection = leftMotor.getDirection();
	bool RightWheelDirection = rightMotor.getDirection(); // reversed to mirror the left motor

	// Speed after putting it on Range of 0-100
	double newLeftWheelSpeed = leftMotor.getSpeed()/5; //<-- The new speed of the left wheel
	double newRightWheelSpeed = rightMotor.getSpeed()/5; //<-- The new speed of the right wheel

	// Writing speed to Pwm Pin to Control the Speed
	// PWM takes speed to set power of Motor in %
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

}
<<<<<<< HEAD
=======

>>>>>>> f4be1f7fe7ec625ab11ca576a8fbea1850a071bf

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
	ros::Subscriber sub = nh.subscribe("cmd_vel", FREQUENCY, &messageCallback);	
	//ROS_INFO_STREAM("Left pwm pin: " << leftMotorPwmPin);
	//ROS_INFO_STREAM("Right pwm pin: " << rightMotorPwmPin);
	ros::spin();
<<<<<<< HEAD
//<<<<<<< HEAD
// Equivalent of JetsonGpio  GPIO.Cleanup();
//=======
	ROS_INFO_STREAM("spin");
	// Equivalent of JetsonGpio  GPIO.Cleanup();
//>>>>>>> 5f284dfaa64ed6777c4ebb61016d90093bfa7cbf
=======

	ROS_INFO_STREAM("spin");
	// Equivalent of JetsonGpio  GPIO.Cleanup();
>>>>>>> f4be1f7fe7ec625ab11ca576a8fbea1850a071bf
	softPwmWrite(LEFT_PWM_PIN,0);
	digitalWrite(LEFT_PWM_PIN,LOW);
	softPwmWrite(RIGHT_PWM_PIN,0);
	digitalWrite(RIGHT_PWM_PIN,LOW);
	digitalWrite(RIGHT_DIRECTION_PIN,LOW);
	digitalWrite(LEFT_DIRECTION_PIN,LOW);

	ROS_INFO_STREAM("clean");
}
