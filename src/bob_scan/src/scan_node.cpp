#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <stdint.h>
#include <thread>
#include <cmath>
#include <filesystem> // For filesystem operations
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/time_source.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <control_msgs/msg/interface_value.hpp>

#include <std_msgs/msg/string.hpp> // For keyboard input

using std::placeholders::_1;

class ScanNode : public rclcpp::Node
{
public:
    ScanNode()
        : Node("scan_node")
    {
        // Implementing the Subscriber for the LaserScan
        scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&ScanNode::scan_callback, this, _1));
        
        // Subscriber for the keyboard input
        keyboard_subscriber = this->create_subscription<std_msgs::msg::String>(
            "keyboard_input", 10, std::bind(&ScanNode::keyboard_callback, this, _1));
    }

private:
    bool capture = false;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_msg;

    void scan_callback(const sensor_msgs::msg::LaserScan &scan_msg)
    {       
        last_scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>(scan_msg);
    }

    void keyboard_callback(const std_msgs::msg::String &key_msg)
    {
        if (key_msg.data.find('o') != std::string::npos && last_scan_msg) // Check if the string contains 'c'
        {
            save_scan_data_to_csv(last_scan_msg);
        }
    }

    void save_scan_data_to_csv(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        std::string directory = "src/bob_scan/csv";
        std::filesystem::create_directories(directory); // Create the directory if it doesn't exist

        std::string file_path = directory + "/scan_data.csv";
        std::ofstream file(file_path);

        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing");
            return;
        }

        file << "Theta/dgree,Ranges/m\n";

        int num_ranges = scan_msg->ranges.size();
        float angle_increment = scan_msg->angle_increment;

        for (int i = 0; i < num_ranges; ++i)
        {
            float theta = ((((i * angle_increment))-3)); // [1 ; 360]* 21539.  989589005f /360 +1
            file << theta << "," << scan_msg->ranges[i] << "\n";
        }

        file.close();

        RCLCPP_INFO(this->get_logger(), "Saved scan data to %s", file_path.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_subscriber;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanNode>());
    rclcpp::shutdown();
    return 0;
}
