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
#include <geometry_msgs/msg/quaternion.hpp>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <control_msgs/msg/interface_value.hpp>


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

        // // Implementing the Subscriber for joint_state topic
        // jointstate_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        //     "joint_state", 10, std::bind(&TestNode::jointstate_callback, this, _1));

        // Implementing the Subscriber for dynamic_joint_state topic
        dynamic_jointstate_subscriber = this->create_subscription<control_msgs::msg::DynamicJointState>(
            "dynamic_joint_states", 10, std::bind(&TestNode::dynamic_jointstate_callback, this, _1));


        // Implementing the Publisher cmd_vel topic
        twist_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("diffbot_base_controller/cmd_vel", 10);
    }

private:
    enum State {
        FORWARD,
        FORWARD_2METER,
        BACKWARD,
        YAW,
        COMPLETE
    };

    State current_state = COMPLETE;
    std::vector<double> target_yaws = {1.5708, 0.0, -1.5708, 0.0, 0.0}; // π/2, 0, -π/2, 0, 2π
    uint8_t yaw_index = 0;
    bool complete = false;
    bool check_alm = false;
    bool test_finished = false;
    std::chrono::high_resolution_clock::time_point time_stamp = std::chrono::high_resolution_clock::now();
    bool timer_started = false;

    // void jointstate_callback(const sensor_msgs::msg::JointState &state)
    // {
    //     //Encoder
    // }

    void dynamic_jointstate_callback(const control_msgs::msg::DynamicJointState &d_state)
    {
        if (complete)
        {
            if(!test_finished)
            {
                system("clear");
                RCLCPP_INFO(this->get_logger(),"Press the Emergency Button");

            }


                
            if (((d_state.interface_values[0].values[3]) && (d_state.interface_values[1].values[3])) == 0)
            {
                std::cout << "Left Weehl Alarm  " << d_state.interface_values[0].values[3] <<std::endl;
                std::cout << "Rigth Weehl Alarm  " << d_state.interface_values[1].values[3] <<std::endl;

                RCLCPP_INFO(this->get_logger(),"Alarm Test Completed");
                std::this_thread::sleep_for(std::chrono::seconds(2));
                RCLCPP_INFO(this->get_logger(), "All Tests Completed");
                std::this_thread::sleep_for(std::chrono::seconds(2));
                system("clear");
                RCLCPP_INFO(this->get_logger(), "Pull the Emergency Button and type (y) to finish the Test");
                char response;
                std::cin >> response;
                std::this_thread::sleep_for(std::chrono::seconds(1));

                if ((response == 'y' || response == 'Y'))
                {                
                    RCLCPP_INFO(this->get_logger(), "Test Finished");
                    test_finished = true;
                    std::this_thread::sleep_for(std::chrono::seconds(2));

                }
             }
            if (((d_state.interface_values[0].values[3]) && (d_state.interface_values[1].values[3])) == 1 && test_finished)
            {
                check_alm = true;
            }
        }
    }
    void odom_callback(const nav_msgs::msg::Odometry &odom)
    {


        auto twist_msg = geometry_msgs::msg::TwistStamped();

        switch (current_state) 
        {
            case FORWARD:

                    twist_msg.twist.linear.x = 0.2;
                    std::cout << odom.pose.pose.position.x << std::endl;

                    if (odom.pose.pose.position.x >= 1.0) 
                    {
                        twist_msg.twist.linear.x = 0.0;
                        twist_publisher->publish(twist_msg);
                        std::cout << "Has B.ob driven 1m? (y/n): ";
                        char response;
                        std::cin >> response;
                        if (response == 'y' || response == 'Y') 
                        {
                            RCLCPP_INFO(this->get_logger(),"Forward Test Completed");
                            current_state = BACKWARD;
                        }
                        else 
                        {
                            complete = true;
                            check_alm = true;
                        }
                    }
               
                break;

            case BACKWARD:
              
                    twist_msg.twist.linear.x = -0.2;
                    std::cout << odom.pose.pose.position.x << std::endl;

                    if (odom.pose.pose.position.x <= 0.0) 
                    {
                        twist_msg.twist.linear.x = 0.0;
                        twist_publisher->publish(twist_msg);
                        std::cout << "Has B.ob driven back to start? (y/n): ";
                        char response;
                        std::cin >> response;
                        if (response == 'y' || response == 'Y') 
                        {
                            RCLCPP_INFO(this->get_logger(), "Backward Test Completed");
                            current_state = FORWARD_2METER;
                        }
                        else 
                        {
                            complete = true;
                            check_alm = true;
                        }
                    }
                
                break;
                

            case FORWARD_2METER:
              
                    twist_msg.twist.linear.x = 0.2;
                    std::cout << odom.pose.pose.position.x << std::endl;

                    if (odom.pose.pose.position.x >= 2.0) 
                    {
                        twist_msg.twist.linear.x = 0.0;
                        twist_publisher->publish(twist_msg);
                        std::cout << "Has B.ob driven 2m? (y/n): ";
                        char response;
                        std::cin >> response;
                        if (response == 'y' || response == 'Y') 
                        {
                            RCLCPP_INFO(this->get_logger(),"Forward 2m Test Completed");
                            current_state = YAW;
                        }
                        else 
                        {
                            complete = true;
                            check_alm = true;
                        }
                    }
               
                break;
                

            case YAW:
                if (yaw_index < target_yaws.size() && !complete) 
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

                    // yaw = fmod(yaw,M_2_PI);

                    // if (yaw < 0){
                    //     yaw += M_2_PI;
                    // }

                    // Print the yaw angle
                    // RCLCPP_INFO(this->get_logger(), "Yaw: %f", yaw);

                    // Check if yaw angle is approximately the target yaw

                    if (yaw_index == 0 || yaw_index == 3 )
                    {
                         twist_msg.twist.angular.z = 0.1;   
                    }
                    else
                    {
                         twist_msg.twist.angular.z = -0.1;   
                    }

                    if (yaw_index == 4 && !timer_started)
                    {
                        time_stamp = std::chrono::high_resolution_clock::now();
                        timer_started = true;

                    }
                    
                    RCLCPP_INFO(this->get_logger(), "D_Yaw: %f", (std::abs(yaw - target_yaws[yaw_index])));

                    std::chrono::duration<double> diff = std::chrono::high_resolution_clock::now() - time_stamp;
                    
                    if (std::abs(yaw - target_yaws[yaw_index]) <= 0.03 && diff.count() > 0.3) 
                    {
                        twist_msg.twist.angular.z = 0.0;
                        twist_publisher->publish(twist_msg);
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        std::cout << "Has B.ob achieved yaw " << (int)((target_yaws[yaw_index]*57.29564553f)) << " degree ? (y/n): ";
                        char response;
                        std::cin >> response;
                        if (response == 'y' || response == 'Y') 
                        {

                            RCLCPP_INFO(this->get_logger(), "Yaw Test Completed: Yaw is approximately %d", (int)((target_yaws[yaw_index]*57.29564553f)));
                            yaw_index++;
                            if (yaw_index >= target_yaws.size()) 
                            {
                                current_state = COMPLETE;
                            }
                        }
                        else 
                        {
                            complete = true;
                            check_alm = true;
                        }
                    }
                } 

                else if (yaw_index >= target_yaws.size()) 
                {
                    current_state = COMPLETE;
                }
                break;

            case COMPLETE:

                complete = true;
                break;
        }


        if (!complete)
        {
            twist_publisher->publish(twist_msg);
        }

        if (complete && check_alm )
        {
            
            // Launch diffbot.launch.py and exit this node
            RCLCPP_INFO(this->get_logger(), "Close the Node");
            rclcpp::shutdown();
        }

        
        
    
    }

    //! Subscriber to read from the odom topic
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;

    //! Subscriber to read from the joint_state topic
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointstate_subscriber;

    //! Subscriber to read from the dynamic_joint_state topic
    rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr dynamic_jointstate_subscriber;

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
