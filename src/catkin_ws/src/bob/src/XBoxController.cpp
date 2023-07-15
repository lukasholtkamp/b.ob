/**
 * @brief This program is used to control the robot using XBoxOne Controller
 * @details This program is used to control the robot using XBoxOne Controller. The program is used to control the robot in the following way:
 * 1. The left joystick is used to control either the rotating on the point or left and right movement of the robot.
 * 2. The rigt trigger is used to control the forward movement of the robot (accelerate).
 * 3. The left trigger is used to control the backward movement of the robot (decelerate).
 * 4. The A button is used to pause the movement.
 * 5. The B button is used to stop the movement.
 */
#include <XBoxController.h>

// Class reads Controller Inputs and calculates Velocity then publishes it to RunMotor.cpp
/**
 * @brief This class is used to control the robot using XBoxOne Controller
 * @details Class reads Controller Inputs and calculates Velocity then publishes it to RunMotor.cpp 
 */
class XBoxController
{
public:
	XBoxController();

private:
	/**
   * @brief Callback function for the joystick
   * @param joy 
   */
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle nh; //<-- Node handle

	int linear, angular; //<-- Linear and angular values
	int bButton; //<-- Button value
	double lScale, aScale; //<-- Scale values
	ros::Publisher velPub; //<-- Publisher
	ros::Subscriber joy_sub; //<-- Subscriber
};

/**
 * @brief Constructor
 * @details This constructor is used to initialize the XBoxController class. The constructor is used to set the linear and angular values, the scale values and the button value.
 * 
 */
XBoxController::XBoxController():
	linear(JOY_AXIS_RT), 
	angular(ANGULAR_VEL)
{
  // Getting the parameters from the parameter server
	nh.param("axis_linear", linear, linear);
	nh.param("axis_angular", angular, angular);
	nh.param("scale_angular", aScale, aScale);
	nh.param("scale_linear", lScale, lScale);

	velPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10); //<-- Publisher

	joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &XBoxController::joyCallback, this); //<-- Subscriber
}

/**
 * @brief Callback function for the joystick
 * @details This function is used to get the joystick values and publish the linear and angular values to the robot.
 * 
 * @param joy 
 */
void XBoxController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
// Getting the linear and angular values
  geometry_msgs::Twist twist; //<-- Twist message
  twist.angular.z = aScale*joy->axes[ANGULAR_VEL]; //<-- Angular value
  int val = 0; 
  switch(val){ // Switch case to control the linear value
	case JOY_AXIS_RT: // If the right trigger is pressed
		twist.linear.x = ((1+(lScale*joy->axes[JOY_AXIS_RT])*-1)/2); // Setting the linear value
		break;
	case JOY_AXIS_LT: // If the left trigger is pressed
		twist.linear.x = ((1+(lScale*joy->axes[JOY_AXIS_LT])*-1)/2); // Setting the linear value
		break;
	default: // If no trigger is pressed
		twist.linear.x = ((1+(lScale*joy->axes[JOY_AXIS_RT])*-1)/2)-((1+(lScale*joy->axes[JOY_AXIS_LT])*-1)/2); // Setting the linear value
  }
  
// Pause and Stop Buttons
  auto bButton = joy->buttons[B_BUTTON]; // Getting the value of the B button
  auto yButton = joy->buttons[Y_BUTTON]; // Getting the value of the Y button
  if(bButton == 1){ // If the B button is pressed
  	twist.linear.x = 0; 
  	twist.angular.z = 0;
	velPub.publish(twist); // Publishing the linear and angular values
	//ros::shutdown(); // Shutting down the node
	system("rosnode kill -a");
  }
  if(yButton == 1){ // If the Y button is pressed
  	twist.linear.x = 0;
  	twist.angular.z = 0;
  }
  velPub.publish(twist); // Publishing the linear and angular values
}

/**
 * @brief Main function
 * @details This is the main function of the program. This function is used to initialize the ros node and the XBoxController class.
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "xbox_controller"); //<-- Initializing the ros node
  XBoxController xbox_controller; //<-- Initializing the XBoxController class
  ros::spin(); //<-- Spin
}
