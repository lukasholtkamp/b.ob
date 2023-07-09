#include <XBoxController.h>

auto lT = 0;
// Class reads Controller Inputs and calculates Velocity then publishes it to RunMotor.cpp
class XBoxController
{
public:
	XBoxController();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle nh;

	int linear, angular;
	int bButton;
	double lScale, aScale;
	ros::Publisher velPub;
	ros::Subscriber joy_sub;
};


XBoxController::XBoxController():
	linear(JOY_AXIS_RT),
	angular(ANGULAR_VEL)
{

	nh.param("axis_linear", linear, linear);
	nh.param("axis_angular", angular, angular);
	nh.param("scale_angular", aScale, aScale);
	nh.param("scale_linear", lScale, lScale);

	velPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &XBoxController::joyCallback, this);
}

void XBoxController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
// Getting the linear and angular values
	geometry_msgs::Twist twist;
	twist.angular.z = aScale*joy->axes[ANGULAR_VEL];

	int val = 0;
// Calculating linear Velocity with acceleration and deceleration
	switch(val){
		case JOY_AXIS_RT:
			twist.linear.x = ((1+(lScale*joy->axes[JOY_AXIS_RT])*-1)/2);
			break;
		case JOY_AXIS_LT:
			twist.linear.x = ((1+(lScale*joy->axes[JOY_AXIS_LT])*-1)/2);
			break;
		default:
			twist.linear.x = ((1+(lScale*joy->axes[JOY_AXIS_RT])*-1)/2)-((1+(lScale*joy->axes[JOY_AXIS_LT])*-1)/2);
	}

// Pause and Stop Buttons
	auto bButton = joy->buttons[B_BUTTON];
	auto yButton = joy->buttons[Y_BUTTON];
// stop Process
	if(bButton == 1){
  		twist.linear.x = 0;
  		twist.angular.z = 0;
		velPub.publish(twist);
		ros::shutdown();
  }
// Set linear and angular velocity to 0 to stop Car
	if(yButton == 1){
  		twist.linear.x = 0;
  		twist.angular.z = 0;
  }
//publishing linear and angular velocity
	velPub.publish(twist);
}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "xbox_controller");
	XBoxController xbox_controller;
	ros::spin();
}
