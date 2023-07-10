#ifndef X_BOX_CONTROLLER_H
#define X_BOX_CONTROLLER_H

#include <ros/ros.h> //<-- Used for the ROS environment
#include <geometry_msgs/Twist.h> //<-- Used to receive messages from the cmd_vel topic
#include <sensor_msgs/Joy.h> //<-- Used to receive messages from the joy topic
#include <ostream> //<-- Used to print to the console

//--Button configuration according to XBoxOne Controller
#define A_BUTTON 0 
#define B_BUTTON 1 
#define Y_BUTTON 4
//--Axes configuration according to XBoxOne Controller
#define ANGULAR_VEL 0 
#define JOY_AXIS_RT 4
#define JOY_AXIS_LT 5


#endif // MOTOR_DRIVER_H

