#include <memory>
#include <string>
#include <iostream>
#include <stdint.h> //<-- Used to define the int32
#include <thread>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

std::string find_button(std::vector<int> buttons);         // <-- Implement find_button function
void launch_call(std::string mode, std::string last_mode); // <-- Implement launch_call function

class DriveMode : public rclcpp::Node
{
public:
    DriveMode()
        : Node("drive_selection")
    {
        // Implementing the Subscriber for the Button request
        gamepad_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&DriveMode::topic_callback, this, _1));

        // Implementing the Subscriber for Drive Mode request
        status_subscriber = this->create_subscription<std_msgs::msg::String>(
            "drive_mode_status", 10, std::bind(&DriveMode::status_callback, this, _1));

        // Implementing the Publisher for the Drive Mode Status
        status_publisher = this->create_publisher<std_msgs::msg::String>("drive_mode_status", 10);
    }

    std::string last_mode = "IDLE"; // <-- Implementing the las_mode variable for the drive_mode_status request

private:
    // Button callback function
    void topic_callback(const sensor_msgs::msg::Joy &joy_msg)
    {
        auto drive_mode_status_msg = std_msgs::msg::String();

        // Read button input, transfer it to the button output function and write it to the drive mode status
        if (find_button(joy_msg.buttons) == "")
        {
            drive_mode_status_msg.data = last_mode;
        }
        else
        {
            drive_mode_status_msg.data = find_button(joy_msg.buttons);
        }

        // Transfers the drive mode status to the publisher
        status_publisher->publish(drive_mode_status_msg);
    }

    // Drive Mode Status callback funtion
    void status_callback(const std_msgs::msg::String &drive_mode_status_msg)
    {
        // Read the drive mode status, output in the terminal and transfer it to the launch function
        if (last_mode != drive_mode_status_msg.data)
        {
            std::cout << "Driving Mode: " << drive_mode_status_msg.data << std::endl;

            launch_call(drive_mode_status_msg.data, last_mode);
            last_mode = drive_mode_status_msg.data;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr gamepad_subscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscriber;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher;
};

/**
 * @brief Gets and read the button input
 *
 * @return Which button is pressed and writes it to the terminal
 */
std::string find_button(std::vector<int> buttons)
{

    if (buttons[0] == 1)
    {
        // Button A
        return "Assisted Drive Mode";
    }
    if (buttons[1] == 1)
    {
        // Button B
        // system("killall teleop_node");
        return "Emergency Stop";
    }
    if (buttons[4] == 1)
    {
        // Button Y
        return "Return to Selection";
    }
    if (buttons[3] == 1)
    {
        // Button X
        // system("ros2 launch joy_linux basic_drive.launch.py");
        return "Basic Drive Mode";
    }
    if (buttons[6] == 1)
    {
        // Button LB
        return "Button Left Bumper is pressed!";
    }
    if (buttons[7] == 1)
    {
        // Button RB
        return "Button Right Bumper is pressed!";
    }
    if (buttons[13] == 1)
    {
        // Button LTS
        return "Button Left Thumbstick is pressed!";
    }
    if (buttons[14] == 1)
    {
        // Button RTS
        return "Button Right Thumbstick is pressed!";
    }
    if (buttons[11] == 1)
    {
        // Menu Button
        // system("shutdown now");
        return "Shutdown";
    }
    else
    {
        return "";
    }
}

/**
 * @brief Gets the actual drive mode status
 *
 * @return The several launch or kill system commands
 */
void launch_call(std::string drive_mode_status, std::string last_mode)
{
    if (last_mode == "Basic Drive Mode")
    {
        system("killall teleop_node");
    }
    if (drive_mode_status == "Basic Drive Mode")
    {
        system("ros2 launch teleop_twist_joy teleop-launch.py &");
    }
    if (drive_mode_status == "Shutdown")
    {
        system("shutdown now");
    }
    if (drive_mode_status == "Emergency Stop")
    {
        kill(getppid(), 9);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveMode>());
    rclcpp::shutdown();
    return 0;
}