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
    float Kp;
    float Ki;
    float Kd;
    float error;
    float setpoint;
    float sum;
    float dt;
    float previous_error;
    float max_output;
    float min_output;
    float max_sum;
    float min_sum;
    const float tolerance = 0.01;

    float prev_setpoint;

    rclcpp::Time last_time;

    tf2::Quaternion initialpose_orientation;

    void initialize_pid(float Kp, float Ki, float Kd, float min_output, float max_output, float min_sum, float max_sum);
    float normalize_angle(float angle);
    void setpoint_callback(const std_msgs::msg::Float32 &setpoint_msg);
    void joy_callback(const sensor_msgs::msg::Joy &joy_msg);
    void odom_callback(const nav_msgs::msg::Odometry &odom);
    void publish_initial_pose(double x, double y, const tf2::Quaternion &quaternion);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr setpoint_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr gamepad_subscriber;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pid_cmd_publisher;
};