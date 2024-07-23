#pragma once

#include <memory>
#include <string>
#include <iostream>
#include <stdint.h>
#include <thread>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_client.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <control_msgs/msg/interface_value.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "std_msgs/msg/float32.hpp"

class PID : public rclcpp::Node
{
public:
    PID();

private:
    float Kp;                         // Proportional gain
    float Ki;                         // Integral gain
    float Kd;                         // Derivative gain
    float error;                      // Error between the setpoint and the measured point
    float setpoint;                   // Setpoint
    float sum;                        // Error sum for the integral part
    float dt;                         // Time difference used for the integral and derivative parts
    float previous_error;             // Previous error for the derivative part
    float max_output;                 // Maximum allowed output
    float min_output;                 // Minimum allowed output
    float max_sum;                    // Maximum allowed sum
    float min_sum;                    // Minimum allowed sum
    const float tolerance = 0.01;     // Tolerance value for deadband
    float prev_setpoint;              // Previous setpoint used to check if there is a new one
    rclcpp::Time last_time;           // Used for calculating dt

    tf2::Quaternion setpoint_pose;    // Used by Rviz to indicate where the setpoint is

    //! Function to initialize the PID controller with parameters from a YAML file
    void initialize_pid(float Kp, float Ki, float Kd, float min_output, float max_output, float min_sum, float max_sum);
    
    //! Function to normalize the measured angle between -π and π
    float normalize_angle(float angle);
    
    //! Callback function to read the setpoint value from teleop_twist_joy.cpp
    void setpoint_callback(const std_msgs::msg::Float32 &setpoint_msg);
    
    //! Callback function for the gamepad subscriber to handle joystick inputs (buttons and axes)
    void joy_callback(const sensor_msgs::msg::Joy &joy_msg);
    
    //! Callback function for calculating the output of the PID controller
    void odom_callback(const nav_msgs::msg::Odometry &odom);

    //! Callback function to publish the setpoint
    void publish_initial_pose(double x, double y, const tf2::Quaternion &quaternion);

    //! Subscriber to read from the odometry/filtered topic to get the orientation
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    
    //! Subscriber to read from the setpoint topic to get the new setpoint 
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr setpoint_subscriber;

    //! Subscriber to read from the joy topic to detect which buttons have been pressed
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr gamepad_subscriber;
    
    //! Publisher to publish to the diffbot_base_controller/cmd_vel_unstamped topic
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;
    
    //! Publisher to publish to the initial_pose topic
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher;

    //! Publisher to publish to the pid_cmd_vel topic to publish the output of the PID controller
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pid_cmd_publisher;
};
