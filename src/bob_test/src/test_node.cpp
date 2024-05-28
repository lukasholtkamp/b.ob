#include <memory>
#include <string>
#include <iostream>
#include <stdint.h> //<-- Used to define the int32
#include <thread>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;

/*! Class for changing the different driving modes e.g. Manual Driving, Autonomous Driving */
class TestMode : public rclcpp::Node
{
public:
    TestMode()
        : Node("test_mode")
    {

        // Implementing the Subscriber for Odometry

        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "diffbot_base_controller/odom", 10, std::bind(&TestMode::odom_callback, this, _1));

        // Implementing the Subscriber for Drive Mode request
        jointstate_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_state", 10, std::bind(&TestMode::jointstate_callback, this, _1));

        // Implementing the Publisher for the Drive Mode Status
        twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>("diffbot_base_controller/cmd_vel", 10);
    }

private:
    /**
     * @brief Callback function called from the jointstate_subscriber which publishes a command to the cmd_vel topic
     *
     */
    void jointstate_callback(const sensor_msgs::msg::JointState &state)
    {
        
        
    }

     void odom_callback(const nav_msgs::msg::Odometry &odom)
    {
        // For demonstration, publish a cmd_vel message with linear.x = 1.0
        auto twist_msg = geometry_msgs::msg::Twist();
        // while (odom.pose.pose.position.x > 1.0)
        // {
        //     // state.position[0] != 1.0f;
        //     twist_msg.linear.x = 0.2;

        // }
        
        twist_msg.linear.x = 0.2;
        twist_publisher->publish(twist_msg);
    }


    //! Subscriber to read from the odom topic
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;

    //! Subscriber to read from the joint_state topic
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointstate_subscriber;

    //! Publisher to publish to the cmd_vel topic
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestMode>());
    rclcpp::shutdown();
    return 0;
}
