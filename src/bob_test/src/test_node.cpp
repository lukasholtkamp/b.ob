#include <memory>
#include <string>
#include <iostream>
#include <stdint.h> 
#include <thread>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
// #include the LCD header when it's done to print the messages on it

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>                   
#include <geometry_msgs/msg/quaternion.hpp>            // msg_type to handle with Quarternion
#include <control_msgs/msg/dynamic_joint_state.hpp>    // msg_type for reading the alarm state
#include <control_msgs/msg/interface_value.hpp>        // msg_type to alow the converting from Quaternion to an Euler angle
#include <sensor_msgs/msg/joy.hpp>


using std::placeholders::_1;

class TestNode : public rclcpp::Node
{
public:
    TestNode()
        : Node("test_node")
    {
        // Implementing the Subscriber for odom topic
        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "diffbot_base_controller/odom", 10, std::bind(&TestNode::odom_callback, this, _1));

        // Implementing the Subscriber for dynamic_joint_states topic
        dynamic_jointstate_subscriber = this->create_subscription<control_msgs::msg::DynamicJointState>(
            "dynamic_joint_states", 10, std::bind(&TestNode::dynamic_jointstate_callback, this, _1));

        // Implementing the Subscriber for the Button and Axes request
        gamepad_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&TestNode::joy_callback, this, _1));


        // Implementing the Publisher for the cmd_vel topic
        twist_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("diffbot_base_controller/cmd_vel", 10);
    }

private:
        // The Test States  
    enum State {
        FORWARD,
        FORWARD_2METER,
        BACKWARD,
        YAW,
        COMPLETE
    };

    //! Enum variable for the test state
    State current_state = FORWARD;
    //! Vector of target yaw values
    std::vector<double> target_yaws = {1.5708, 0.0, -1.5708, 0.0, 0.0}; // π/2, 0, -π/2, 0, 2π
    //! Iterator for the target yaw values
    uint8_t yaw_index = 0;
    //! Boolean to know when each driving test is finished
    bool driving_test_complete = false;
    //! Boolean to know when alarm test is complete
    bool alm_complete = false;
    //! Boolean to know when all tests are complete
    bool tests_finished = false;
    //! Time stamp used for doing the 360 degree turn
    std::chrono::high_resolution_clock::time_point time_stamp = std::chrono::high_resolution_clock::now();
    //! Boolean to know when to start timer
    bool timer_started = false;
    //! Boolean to abort tests
    bool test_break = false;
    //! Boolean to move on to next driving test
    bool next_test = false;

    
    // Callback function to read the Inputs (Buttons and Axes)
    void joy_callback(const sensor_msgs::msg::Joy &joy_msg)
    {
        if (joy_msg.axes[6] == 1.0)
        {
            test_break = true;
        }
        if (joy_msg.axes[6] == -1.0)
        {
            next_test = true;
        }
    }

    // Callback function to test the Alarm
    void dynamic_jointstate_callback(const control_msgs::msg::DynamicJointState &d_state)
    {
        if (driving_test_complete)
        {
            if(!tests_finished)
            {
                system("clear");
                std::cout << "Press the Emergency Button" << std::endl;                
                std::cout << "\nNote: If the Button is pressed and nothing has changed press Left-D-PAD button to escape" << std::endl;

            }

            // Check if the Emergency Button is pressed
            if (((d_state.interface_values[0].values[3]) && (d_state.interface_values[1].values[3])) == 0)
            {
                system("clear");
                std::cout << "Alarm Test Completed" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(3));
                std::cout << "Release the Emergency Button" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(3));
                tests_finished = true;
             }

            // Check if the Emergency Button is released and finish the test
            if (((d_state.interface_values[0].values[3]) && (d_state.interface_values[1].values[3])) == 1 && tests_finished)
            {

                    std::cout <<"Test Finished" << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                    alm_complete = true;
                
            }
        }
        // Check if the Left-D-PAD Button is pressed to abort the test
        if (test_break)
        {
            std::cout << "\nAlarm Test Failed" << std::endl;
            alm_complete = true;
        }
    }

    // Callback funktion to test B.ob in all directions 
    void odom_callback(const nav_msgs::msg::Odometry &odom)
    {


        auto twist_msg = geometry_msgs::msg::TwistStamped();

        switch (current_state) 
        {
            case FORWARD:

                    // Move B.ob forward with velocity_command value 0.2
                    twist_msg.twist.linear.x = 0.2;
                    std::cout << odom.pose.pose.position.x << std::endl;

                    // Check if B.ob has reached 1 meter
                    if (odom.pose.pose.position.x >= 1.0) 
                    {
                        // Stop publishing
                        twist_msg.twist.linear.x = 0.0;
                        twist_publisher->publish(twist_msg);
                        system("clear");
                        std::cout << "Has B.ob driven 1m? \nPress Right-D-PAD for Yes , Left-D-PAD for No : " << std::endl;
                        std::this_thread::sleep_for(std::chrono::seconds(3));

                        // Check if Right-D-PAD Button is pressed to continue the test
                        if (next_test) 
                        {
                            std::cout << "Forward Test Completed"<< std::endl;
                            std::this_thread::sleep_for(std::chrono::seconds(2));
                            next_test =false;
                            current_state = BACKWARD;
                        }

                        // Check if the Left-D-PAD Button is pressed to abort the test
                        else if (test_break)
                        {
                            current_state = COMPLETE;
                        }
                    }
               
                break;

            case BACKWARD:

                    // Move B.ob backward with velocity_command value -0.2
                    twist_msg.twist.linear.x = -0.2;
                    std::cout << odom.pose.pose.position.x << std::endl;

                    // Check if B.ob has reached the start position
                    if (odom.pose.pose.position.x <= 0.0) 
                    {
                        // Stop publishing
                        twist_msg.twist.linear.x = 0.0;
                        twist_publisher->publish(twist_msg);
                        system("clear");
                        std::cout << "Has B.ob driven back to start? \nPress right-D-PAD for Yes , Left-D-PAD for No : " << std::endl;
                        std::this_thread::sleep_for(std::chrono::seconds(3));

                        // Check if Right-D-PAD Button is pressed to continue the test
                        if (next_test) 
                        {
                            std::cout << "Backward Test Completed"<< std::endl;
                            std::this_thread::sleep_for(std::chrono::seconds(2));
                            next_test =false;
                            current_state = FORWARD_2METER;
                        }

                        // Check if the Left-D-PAD Button is pressed to abort the test
                        else if (test_break)
                        {
                            current_state = COMPLETE;
                        }
                    }
                
                break;
                

            case FORWARD_2METER:

                    // Move B.ob forward with velocity_command value 0.2
                    twist_msg.twist.linear.x = 0.2;
                    std::cout << odom.pose.pose.position.x << std::endl;

                    // Check if B.ob has reached 2 meters
                    if (odom.pose.pose.position.x >= 2.0) 
                    {
                        // Stop publishing
                        twist_msg.twist.linear.x = 0.0;
                        twist_publisher->publish(twist_msg);
                        system("clear");
                        std::cout << "Has B.ob driven 2m? \nPress right-D-PAD for Yes , Left-D-PAD for No : " << std::endl;
                        std::this_thread::sleep_for(std::chrono::seconds(3));

                        // Check if Right-D-PAD Button is pressed to continue the test
                        if (next_test) 
                        {
                            std::cout << "Forward 2m Test Completed"<< std::endl;
                            std::this_thread::sleep_for(std::chrono::seconds(2));
                            next_test =false;
                            current_state = YAW;
                        }

                        // Check if the Left-D-PAD Button is pressed to abort the test
                        else if (test_break)
                        {
                            current_state = COMPLETE;
                        }
                    }
               
                break;
                

            case YAW:
                
                // Check if the current yaw index is less than the size of target_yaws (5) and the driving test is not complete
                if (yaw_index < target_yaws.size() && !driving_test_complete) 
                {
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
                    
                    // Set angular velocity based on yaw_index
                    if (yaw_index == 0 || yaw_index == 3 )
                    {
                         twist_msg.twist.angular.z = 0.1;           // Move B.ob to the right    
                    }
                    else
                    {
                         twist_msg.twist.angular.z = -0.1;          // Move B.ob to the left   
                    }

                    // Start timer for the last yaw (360°) to make a small adjustment before checking the condition
                    if (yaw_index == 4 && !timer_started)
                    {
                        time_stamp = std::chrono::high_resolution_clock::now();
                        timer_started = true;

                    }
                    
                    // Calculate the time difference
                    std::chrono::duration<double> diff = std::chrono::high_resolution_clock::now() - time_stamp;
                    
                    // Check if the current yaw is within the target range and a sufficient amount of time has passed
                    if (std::abs(yaw - target_yaws[yaw_index]) <= 0.03 && diff.count() > 0.3) 
                    {
                        // Stop publishing
                        twist_msg.twist.angular.z = 0.0;
                        twist_publisher->publish(twist_msg);
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        system("clear");
                        std::cout << "Has B.ob achieved yaw " << (int)((target_yaws[yaw_index]*57.29564553f)) << " degree ? \nPress right-D-PAD for Yes , Left-D-PAD for No : " << std::endl;
                        std::this_thread::sleep_for(std::chrono::seconds(3));

                        // Check if Right-D-PAD Button is pressed to continue the test
                        if (next_test) 
                        {
                            next_test =false;
                            yaw_index++;
                            
                            // Check if the Left-D-PAD Button is pressed to abort the test
                            if (yaw_index >= target_yaws.size()) 
                            {
                                current_state = COMPLETE;
                            }
                        }

                        // Check if the Left-D-PAD Button is pressed to abort the test
                        else if (test_break)
                        {
                            current_state = COMPLETE;
                        }
                    }
                } 
                
                // Check if the current yaw index has reached the size of target_yaws to finish the driving test
                else if (yaw_index >= target_yaws.size()) 
                {
                    current_state = COMPLETE;
                }
                break;

            case COMPLETE:

                driving_test_complete = true;
                break;
        }


        if (!driving_test_complete)
        {
            // Publish
            twist_publisher->publish(twist_msg);

        // Check if the Left-D-PAD Button is pressed to abort the driving test
        if (test_break)
        {
            std::cout << "Driving Test Failed" << std::endl;
            driving_test_complete = true;
            test_break = false;
        }

        }

        if (driving_test_complete && alm_complete )
        {
            // Closes the Node
            // TODO: Launch diffbot.launch.py and exit this node
            std::cout << "Close the Node" << std::endl;
            rclcpp::shutdown();
        }

        
     
    }

    //! Subscriber to read from the odom topic
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;

    //! Subscriber to read from the joint_states topic
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointstate_subscriber;

    //! Subscriber to read from the dynamic_joint_states topic
    rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr dynamic_jointstate_subscriber;

    //! Publisher to publish to the cmd_vel topic
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher;

    //! Subscriber to read from the joy topic to know which buttons have been pressed
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr gamepad_subscriber;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestNode>());
    rclcpp::shutdown();
    return 0;
}
