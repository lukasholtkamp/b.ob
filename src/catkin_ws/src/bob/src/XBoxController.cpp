/**
 * @brief This program is used to control the robot using XBoxOne Controller
 * @details This program is used to control the robot using XBoxOne Controller. The program is used to control the robot in the following way:
 * 1. The left joystick is used to control either the rotating on the point or left and right movement of the robot.
 * 2. The rigt trigger is used to control the forward movement of the robot (accelerate).
 * 3. The left trigger is used to control the backward movement of the robot (decelerate).
 * 4. The A button is used to pause the movement.
 * 5. The B button is used to stop the movement.
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <ostream>

//--Button configuration according to XBoxOne Controller
#define A_BUTTON 0
#define B_BUTTON 1
#define Y_BUTTON 3
//--Axes configuration according to XBoxOne Controller
#define ANGULAR_VEL 0
#define JOY_AXIS_RT 5
#define JOY_AXIS_LT 2

/**
 * @brief This class is used to control the robot using XBoxOne Controller
 * 
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
  int b_button; //<-- Button value
  double l_scale, a_scale;  //<-- Scale values
  ros::Publisher vel_pub; //<-- Publisher
  ros::Subscriber joy_sub; //<-- Subscriber
};

/**
 * @brief Constructor
 * @details This constructor is used to initialize the XBoxController class. The constructor is used to set the linear and angular values, the scale values and the button value.
 * 
 */
XBoxController::XBoxController():
  linear(JOY_AXIS_RT), //linear(AXIS_LINEAR),
  angular(ANGULAR_VEL), //angular(AXIS_ANGULAR),
  b_button(B_BUTTON) //b_button(BUTTONS)
{

  // Getting the parameters from the parameter server
  nh.param("axis_linear", linear, linear); 
  nh.param("axis_angular", angular, angular);
  nh.param("scale_angular", a_scale, a_scale);
  nh.param("scale_linear", l_scale, l_scale);
  nh.param("buttons", b_button, b_button);

  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10); //<-- Publisher

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
  twist.angular.z = a_scale*joy->axes[ANGULAR_VEL]; //<-- Angular value
  int val = 0; 
  switch(val){ // Switch case to control the linear value
	case JOY_AXIS_RT: // If the right trigger is pressed
		twist.linear.x = ((1+(l_scale*joy->axes[JOY_AXIS_RT])*-1)/2); // Setting the linear value
		break;
	case JOY_AXIS_LT: // If the left trigger is pressed
		twist.linear.x = ((1+(l_scale*joy->axes[JOY_AXIS_LT])*-1)/2); // Setting the linear value
		break;
	default: // If no trigger is pressed
		twist.linear.x = ((1+(l_scale*joy->axes[JOY_AXIS_RT])*-1)/2)-((1+(l_scale*joy->axes[JOY_AXIS_LT])*-1)/2); // Setting the linear value
  }
  
// Pause and Stop Buttons
  auto b_button = joy->buttons[B_BUTTON]; // Getting the value of the B button
  auto y_button = joy->buttons[Y_BUTTON]; // Getting the value of the Y button
  if(b_button == 1){ // If the B button is pressed
  	twist.linear.x = 0; 
  	twist.angular.z = 0;
	vel_pub.publish(twist); // Publishing the linear and angular values
	ros::shutdown(); // Shutting down the node
  }
  if(y_button == 1){ // If the Y button is pressed
  	twist.linear.x = 0;
  	twist.angular.z = 0;
  }
  vel_pub.publish(twist); // Publishing the linear and angular values
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
