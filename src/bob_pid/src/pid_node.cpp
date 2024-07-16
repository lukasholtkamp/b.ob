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
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <control_msgs/msg/interface_value.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

class PID : public rclcpp::Node
{
public:
    PID()
        : Node("pid_node"), pid_initialized(false), dt(0.1), last_time(this->now())
    {
        // Implementing the Subscriber for odom topic
        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&PID::odom_callback, this, _1));

        // Implementing the Subscriber for the Button and Axes request
        gamepad_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&PID::joy_callback, this, _1));

        // Implementing the Subscriber for the Button and Axes request
        setpoint_subscriber = this->create_subscription<std_msgs::msg::Float32>(
            "setpoint", 10, std::bind(&PID::setpoint_callback, this, _1));

        // Implementing the Publisher for the cmd_vel topic
        twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>("diffbot_base_controller/cmd_vel_unstamped", 10);

        // Implementing the Publisher for the initial_pose topic
        initial_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initial_pose", 10);

        pid_cmd_publisher = this->create_publisher<std_msgs::msg::Float32>("pid_cmd_vel", 10);

        // Initialize the PID controller
        initialize_pid(0.7, 0.0, 0.1, -0.4, 0.4, -2.0, 2.0, false); // Update with anti-windup and output clamping limits
    }

private:
    bool pid_on;
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
    const float tolerance = 0.01; // Tolerance range for the deadband

    bool pid_initialized;
    rclcpp::Time last_time; // To store the time of the last callback

    tf2::Quaternion initialpose_orientation;

    /**
     * @brief Initialize the PID controller with the given parameters.
     *
     * @param Kp Proportional gain
     * @param Ki Integral gain
     * @param Kd Derivative gain
     * @param min_output Minimum output value
     * @param max_output Maximum output value
     * @param min_sum Minimum integral sum value
     * @param max_sum Maximum integral sum value
     */
    void initialize_pid(float Kp, float Ki, float Kd, float min_output, float max_output, float min_sum, float max_sum, bool pid_on)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->error = 0;
        this->setpoint = 0;
        this->sum = 0;
        this->previous_error = 0;
        this->min_output = min_output;
        this->max_output = max_output;
        this->min_sum = min_sum;
        this->max_sum = max_sum;
        this->pid_on = pid_on;
    }

    /**
     * @brief Normalize the angle to be within the range [-π, π].
     *
     * @param angle The angle to be normalized.
     * @return Normalized angle within [-π, π].
     */
    float normalize_angle(float angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    /**
     * @brief Callback function for the gamepad_subscriber to handle joystick inputs (Buttons and Axes).
     *
     * Updates the setpoint based on the joystick input.
     *
     * @param joy_msg The message containing joystick data.
     */
    void setpoint_callback(const std_msgs::msg::Float32 &setpoint_msg)
    {
        setpoint = setpoint_msg.data;
    }

    void joy_callback(const sensor_msgs::msg::Joy &joy_msg)
    {
        if (joy_msg.axes[0] < -0.05 || joy_msg.axes[0] > 0.05)
        {
            pid_on = false;
            pid_initialized = false;
        }
        else
        {
            pid_on = true;
        }
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

     
        // Round setpoint and yaw to 2 decimal places
        setpoint = round(setpoint * 100.0) / 100.0;
        yaw = round(yaw * 100.0) / 100.0;

        std::cout << "setpoint : " << setpoint << std::endl;
        std::cout << "yaw : " << yaw << std::endl;

        // Normalize yaw and setpoint
        yaw = normalize_angle(yaw);
        setpoint = normalize_angle(setpoint);

        error = setpoint - yaw;
        error = normalize_angle(error); // Ensure the shortest path is taken

        // Round error to 1 decimal place
        error = round(error * 10.0) / 10.0;

        std::cout << "error : " << error << std::endl;

        // Apply deadband to error
        if (std::abs(error) < tolerance)
        {
            error = 0.0;
        }

        sum += error * dt;

        // Round sum to 1 decimal place
        sum = round(sum * 10.0) / 10.0;

        std::cout << "sum : " << sum << std::endl;

        // Implement anti-windup
        if (sum > max_sum)
        {
            sum = max_sum;
        }
        else if (sum < min_sum)
        {
            sum = min_sum;
        }

        float derivative = (error - previous_error) / dt;
        float output = Kp * error + Ki * sum + Kd * derivative;

        // Round output to 1 decimal place
        output = round(output * 10.0) / 10.0;

        std::cout << "output : " << output << "\n\n"
                    << std::endl;

        // Clamp the output
        if (output > max_output)
        {
            output = max_output;
        }
        else if (output < min_output)
        {
            output = min_output;
        }

        previous_error = error;

        // Publish twist message based on PID output
        auto pid_cmd_msg = std_msgs::msg::Float32();

        pid_cmd_msg.data = output;

        pid_cmd_publisher->publish(pid_cmd_msg);
        
    }

    /**
     * @brief Publish the setpoint as an initial pose.
     *
     * @param x The x position of the initial pose.
     * @param y The y position of the initial pose.
     * @param quaternion The quaternion representing the orientation of the initial pose.
     */
    void publish_initial_pose(double x, double y, const tf2::Quaternion &quaternion)
    {
        if (pid_on)
        {
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            pose_msg.header.stamp = this->now(); // Set the timestamp to the current time
            pose_msg.header.frame_id = "odom";
            pose_msg.pose.pose.position.x = x;
            pose_msg.pose.pose.position.y = y;
            pose_msg.pose.pose.orientation.x = quaternion.x();
            pose_msg.pose.pose.orientation.y = quaternion.y();
            pose_msg.pose.pose.orientation.z = quaternion.z();
            pose_msg.pose.pose.orientation.w = quaternion.w();
            initial_pose_publisher->publish(pose_msg);
        }
    }

    //! Subscriber to read from the odom topic
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr setpoint_subscriber;

    //! Publisher to publish to the cmd_vel topic
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;

    //! Publisher to publish the initial_pose
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher;

    //! Subscriber to read from the joy topic to know which buttons have been pressed
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr gamepad_subscriber;

    //! Publisher to publish to the drive_mode_status topic after the driving mode button is pressed
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pid_cmd_publisher;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PID>());
    rclcpp::shutdown();
    return 0;
}

