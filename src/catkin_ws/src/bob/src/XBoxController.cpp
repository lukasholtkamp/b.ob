/**
 * @brief This program is used to control the robot using XBoxOne Controller
 * @details This program is used to control the robot using XBoxOne Controller. The program is used to control the robot in the following way:
 * 1. The left joystick is used to control either the rotating on the point or left and right movement of the robot.
 * 2. The rigt trigger is used to control the forward movement of the robot (accelerate).
 * 3. The left trigger is used to control the backward movement of the robot (decelerate).
 * 4. The A button is used to pause the movement.
 * 5. The B button is used to stop the movement.
 */
#include "motor_driver_library/XBoxController.h"

#include <ostream> //<-- Used to print to the console

#include <geometry_msgs/Twist.h> //<-- Used to receive messages from the cmd_vel topic
#include <ros/ros.h> //<-- Used for the ROS environment
#include <sensor_msgs/Joy.h> //<-- Used to receive messages from the joy topic

/**
 * @brief This class is used to control the robot using XBoxOne Controller
 * @details Class reads Controller Inputs and calculates Velocity then publishes it to RunMotor.cpp 
 */
class XBoxController {
  public:
    XBoxController();

  private:
    /**
     * @brief Callback function for the joystick
     * @param joy 
     */
    void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh; //<-- Node handle

    int linear_, angular_; //<-- Linear and angular values
    double linear_scale_, angular_scale_; //<-- Scale values

    ros::Publisher vel_pub; //<-- Publisher
    ros::Subscriber joy_sub; //<-- Subscriber
};

/**
 * @brief Constructor
 * @details This constructor is used to initialize the XBoxController class. The constructor is used to set the linear and angular values, the scale values and the button value.
 * 
 */
XBoxController::XBoxController() :
	linear_(kJoyAxisRT), 
	angular_(kAngularVel)
{
  // Getting the parameters from the parameter server
	nh.param("axis_linear", linear_, linear_);
	nh.param("axis_angular", angular_, angular_);
	nh.param("scale_angular", angular_scale_, angular_scale_);
	nh.param("scale_linear", linear_scale_, linear_scale_);

	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10); //<-- Publisher

	joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &XBoxController::JoyCallback, this); //<-- Subscriber
}

/**
 * @brief Callback function for the joystick
 * @details This function is used to get the joystick values and publish the linear and angular values to the robot.
 * 
 * @param joy 
 */
void XBoxController::JoyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // Getting the linear and angular values
  geometry_msgs::Twist twist; //<-- Twist message
  twist.angular.z = angular_scale_ * joy -> axes[kAngularVel]; //<-- Angular value
  int val = 0; 
  switch (val) { // Switch case to control the linear value
    case kJoyAxisRT: // If the right trigger is pressed
      twist.linear.x = ((1 + (linear_scale_ * joy -> axes[kJoyAxisRT]) * -1) / 2); // Setting the linear value
      break;
    case kJoyAxisLT: // If the left trigger is pressed
      twist.linear.x = ((1 + (linear_scale_ * joy -> axes[kJoyAxisLT]) * -1) / 2); // Setting the linear value
      break;
    default: // If no trigger is pressed
      twist.linear.x = ((1 + (linear_scale_ * joy -> axes[kJoyAxisRT]) * -1) /2 ) - ((1 + (linear_scale_ * joy -> axes[kJoyAxisLT]) * -1) / 2); // Setting the linear value
  }
  
  // Pause and Stop Buttons
  auto b_button = joy -> buttons[kBButton]; // Getting the value of the B button
  auto y_button = joy -> buttons[kYButton]; // Getting the value of the Y button

  if (b_button == 1) { // If the B button is pressed
  	twist.linear.x = 0; 
  	twist.angular.z = 0;
	  vel_pub.publish(twist); // Publishing the linear and angular values
	  ros::shutdown(); // Shutting down the node
  }
  if (y_button == 1) { // If the Y button is pressed
  	twist.linear.x = 0;
  	twist.angular.z = 0;
  }
  vel_pub.publish(twist); // Publishing the linear and angular values
}

/**
 * @brief Main function
 * @details This is the main function of the program. This function is used to initialize the ros node and the XBoxController class. 
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
