#include "MotorDriver.h"

#include "RunMotor.h"

// motor(directionPin, PwmPin, minSpeed, maxSpeed)
MD::Motor leftMotor(LEFT_DIRECTION_PIN, LEFT_PWM_PIN, MIN_SPEED, MAX_SPEED);
MD::Motor rightMotor(RIGHT_DIRECTION_PIN, RIGHT_PWM_PIN, MIN_SPEED, MAX_SPEED);

// just for debugging
int leftMotorPwmPin = leftMotor.getPwmPin();
int rightMotorPwmPin = rightMotor.getPwmPin();

// Setup PWM for Jetson Nano   (leave in for now!)
// GPIO::PWM leftMotorPWMPin(leftMotorPwmPin, FREQUENCY); // for GPIO::BOARD
// GPIO::PWM rightMotorPWMPin(rightMotorPwmPin, FREQUENCY);


void messageCallback(const geometry_msgs::Twist& cmd_vel) {
// Getting Velocity from XboxController.cpp and calculating Speed and direction
	double linearVelocityX = cmd_vel.linear.x;
	double angularVelocityZ = cmd_vel.angular.z;

	double leftWheelSpeed = (linearVelocityX - angularVelocityZ) * VELOCITY_MULTIPLIER;
	double rightWheelSpeed = (linearVelocityX + angularVelocityZ) * VELOCITY_MULTIPLIER;

	leftMotor.setSpeed(leftWheelSpeed);
	rightMotor.setSpeed(rightWheelSpeed);

	leftMotor.setDirection(leftWheelSpeed, LEFT_DIRECTION_PIN);
        rightMotor.setDirection(-1*rightWheelSpeed, RIGHT_DIRECTION_PIN);

        bool LeftWheelDirection = leftMotor.getDirection();
        bool RightWheelDirection = rightMotor.getDirection(); // reversed to mirror the left motor

// Speed after putting it on Range of 0-100
	double newLeftWheelSpeed = leftMotor.getSpeed();
	double newRightWheelSpeed = rightMotor.getSpeed();




// Writing speed to Pwm Pin to Control the Speed
// PWM takes speed to set power of Motor in %
	//leftMotorPWMPin.ChangeDutyCycle(newLeftWheelSpeed);
	//rightMotorPWMPin.ChangeDutyCycle(newRightWheelSpeed);
	softPwmWrite(LEFT_PWM_PIN,newLeftWheelSpeed);
	softPwmWrite(RIGHT_PWM_PIN,newRightWheelSpeed);

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
/*
void checkTriggers(const sensor_msgs::Joy::ConstPtr& joy){
	auto lT = joy->axes[JOY_AXIS_LT];
        while(lT = 0){

        }
}*/

int main(int argc, char** argv) {
//	leftMotorPWMPin.start(0.0);
//	rightMotorPWMPin.start(0.0);

	ros::init(argc, argv, "run_motor");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("cmd_vel", FREQUENCY, &messageCallback);	

	//ROS_INFO_STREAM("Left pwm pin: " << leftMotorPwmPin);
	//ROS_INFO_STREAM("Right pwm pin: " << rightMotorPwmPin);
	ros::spin();
// Equivalent of JetsonGpio  GPIO.Cleanup();
	softPwmWrite(LEFT_PWM_PIN,0);
	digitalWrite(LEFT_PWM_PIN,LOW);
	softPwmWrite(RIGHT_PWM_PIN,0);
	digitalWrite(RIGHT_PWM_PIN,LOW);
	digitalWrite(RIGHT_DIRECTION_PIN,LOW);
	digitalWrite(LEFT_DIRECTION_PIN,LOW);

	ROS_INFO_STREAM("clean");
}
