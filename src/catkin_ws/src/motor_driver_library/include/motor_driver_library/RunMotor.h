#ifndef RUN_MOTOR_H
#define RUN_MOTOR_H

#include <ros/ros.h> //<-- Used for the ROS environment
#include <geometry_msgs/Twist.h> //<-- Used to receive messages from the cmd_vel topic
#include <sensor_msgs/Joy.h> //<-- Used to receive messages from the joy topic
#include <ros/console.h> //<-- Used to change the verbosity of the program

#define MIN_SPEED               0.0 //<-- The minimum speed of the robot
#define MAX_SPEED               100.0 //<-- The maximum speed of the robot
#define VELOCITY_MULTIPLIER     100.0 //<-- The multiplier used to convert the linear and angular velocities to a percentage
#define FREQUENCY               1000 //<-- The frequency of the PWM signal


#endif // RUN_MOTOR_H

