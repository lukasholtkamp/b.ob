#include <memory>
#include <string>
#include <iostream>
#include <stdint.h>
#include <thread>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <control_msgs/msg/interface_value.hpp>
#include <sensor_msgs/msg/joy.hpp>

using std::placeholders::_1;

class Controller : public rclcpp::Node
{
public:
    Controller()
        : Node("controller_node"), pid_initialized(false), dt(0.1), last_time(this->now())
    {
        // Implementing the Subscriber for odom topic
        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&Controller::odom_callback, this, _1));

        // Implementing the Subscriber for the Button and Axes request
        gamepad_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&Controller::joy_callback, this, _1));

        // Implementing the Publisher for the cmd_vel topic
        twist_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("diffbot_base_controller/cmd_vel", 10);

        // Initialize the PID controller
        initialize_pid(1.0, 0.0, 0.0);
    }

private:
    float Kp;
    float Ki;
    float Kd;
    float error;
    float setpoint;
    float sum;
    float dt;
    float previous_error;

    bool pid_initialized;
    rclcpp::Time last_time; // To store the time of the last callback

    /**
     * @brief Initialize the PID controller with the given parameters.
     *
     * @param Kp Proportional gain
     * @param Ki Integral gain
     * @param Kd Derivative gain
     */
    void initialize_pid(float Kp, float Ki, float Kd)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->error = 0;
        this->setpoint = 0;
        this->sum = 0;
        this->previous_error = 0;
    }

    /**
     * @brief Callback function for the gamepad_subscriber to handle joystick inputs (Buttons and Axes).
     *
     * Updates `test_break` if the D-PAD is pressed left, and `next_test` if the D-PAD is pressed right.
     *
     * @param joy_msg The message containing joystick data.
     */
    void joy_callback(const sensor_msgs::msg::Joy &joy_msg)
    {
        // Implement joystick handling logic here
    }

    /**
     * @brief Callback function to test B.ob's movement in all directions.
     *
     * Uses odometry data to update B.ob's state and publish twist messages.
     *
     * @param odom The message containing odometry data.
     */
    void odom_callback(const nav_msgs::msg::Odometry &odom)
    {
        // Get the current time and calculate dt
        rclcpp::Time current_time = this->now();
        dt = (current_time - last_time).seconds();
        last_time = current_time;

        // Extract quaternion
        auto quaternion = odom.pose.pose.orientation;

        // Convert quaternion to tf2::Quaternion
        tf2::Quaternion tf2_quaternion(
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w);

        // Convert tf2::Quaternion to Euler angles
        tf2::Matrix3x3 mat(tf2_quaternion);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        if (!pid_initialized)
        {
            setpoint = yaw;
            pid_initialized = true;
            RCLCPP_INFO(this->get_logger(), "Setpoint initialized to %f", setpoint);
        }

        error = setpoint - yaw;

        // PID control calculations
        sum += error * dt;
        float derivative = (error - previous_error) / dt;
        float output = Kp * error + Ki * sum + Kd * derivative;
        previous_error = error;

        // Publish twist message based on PID output
        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.stamp = this->now(); // Set the timestamp to the current time
        twist_msg.twist.angular.z = output;
        twist_publisher->publish(twist_msg);
    }

    //! Subscriber to read from the odom topic
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;

    //! Publisher to publish to the cmd_vel topic
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher;

    //! Subscriber to read from the joy topic to know which buttons have been pressed
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr gamepad_subscriber;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
