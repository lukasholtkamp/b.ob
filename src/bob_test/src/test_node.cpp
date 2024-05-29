#include <memory>
#include <string>
#include <iostream>
#include <stdint.h> //<-- Used to define the int32
#include <thread>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;

/*! Class for changing the different driving modes e.g. Manual Driving, Autonomous Driving */
class TestNode : public rclcpp::Node
{
public:
    TestNode()
        : Node("test_node")
    {

        // Implementing the Subscriber for odom topic
        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "diffbot_base_controller/odom", 10, std::bind(&TestNode::odom_callback, this, _1));

        // Implementing the Subscriber for joint_state topic
        jointstate_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_state", 10, std::bind(&TestNode::jointstate_callback, this, _1));

        // Implementing the Publisher cmd_vel topic
        twist_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("diffbot_base_controller/cmd_vel", 10);

        // // Implementing the Publisher odom topic
        // odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("diffbot_base_controller/odom", 10);


    }

    bool test_complete = false;


private:
    /**
     * @brief Callback function called from the jointstate_subscriber which publishes a command to the cmd_vel topic
     *
     */

    void jointstate_callback(const sensor_msgs::msg::JointState &state)
    {
        // if (!(test_complete))
        // {   
        //     std::cout << "LE:  " << state.velocity[0] << "\n"<< "RE:  " << state.velocity[1] << std::endl;
        //     // if ((state.velocity[0] && state.velocity[1]) > 1)
        //     // {
        //     //     system("clear");
        //     //     std::cout << "Encoder test completed" << std::endl;
        //     // }
            
        // }
        
    }

     void odom_callback(const nav_msgs::msg::Odometry &odom)
    {

        auto twist_msg = geometry_msgs::msg::TwistStamped();
        // auto odom_msg = nav_msgs::msg::Odometry();
    if (!test_complete && (odom.pose.pose.position.x == 0))
    {
        twist_msg.twist.linear.x = 0.2;
        twist_publisher->publish(twist_msg);
        test_complete = true;
    }
    
    if (test_complete)
     {
        if (odom.pose.pose.position.x < 1.0f)
        {
            std::cout << (odom.pose.pose.position.x)<< std::endl; 
        }
        else
        {
            // system("clear")
        std::cout << "Test Completed"<< std::endl; 
        twist_msg.twist.linear.x = 0.0;       
        twist_publisher->publish(twist_msg);
        test_complete = false;

        }
       
     }
       
    }
    
    //! Subscriber to read from the odom topic
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;

    //! Subscriber to read from the joint_state topic
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointstate_subscriber;

    //! Publisher to publish to the odom topic
    // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;

    //! Publisher to publish to the cmd_vel topic
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestNode>());
    rclcpp::shutdown();
    return 0;
}
