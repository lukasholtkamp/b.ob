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

/*! Class for changing the different driving modes e.g. Manual Driving, Autonomous Driving*/
class DriveMode : public rclcpp::Node
{
public:
    DriveMode()
        : Node("drive_selection")
    {
        // Implementing the Subscriber for the Button request
        gamepad_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&DriveMode::joy_callback, this, _1));

        // Implementing the Subscriber for Drive Mode request
        status_subscriber = this->create_subscription<std_msgs::msg::String>(
            "drive_mode_status", 10, std::bind(&DriveMode::status_callback, this, _1));

        // Implementing the Publisher for the Drive Mode Status
        status_publisher = this->create_publisher<std_msgs::msg::String>("drive_mode_status", 10);
    }

    //! string to know when there is a transition in driving mode
    std::string last_mode = "IDLE"; // <-- Implementing the las_mode variable for the drive_mode_status request

private:
    /**
     * @brief Callback function called from the gamepad_subscriber which calls find_button to find which button was pressed and publishes this on the drive_mode_status topic
     *
     */
    void joy_callback(const sensor_msgs::msg::Joy &joy_msg)
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

    /**
     * @brief Callback function called from the status_subscriber which calls launch_call when the driving mode changes and updates the last driving mode
     *
     */
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
            return "Basic Drive Mode";
        }
        if (buttons[6] == 1)
        {
            // Button LB
            return "";
        }
        if (buttons[7] == 1)
        {
            // Button RB
            return "";
        }
        if (buttons[13] == 1)
        {
            // Button LTS
            return "";
        }
        if (buttons[14] == 1)
        {
            // Button RTS
            return "";
        }
        
        if (buttons[10] == 1)
        {
            // Button back
            return "Test mode";
        }
        
        if (buttons[11] == 1)
        {
            // Menu Button
            return "Shutdown";
        }
        else
        {
            return "";
        }
    }

    /**
     * @brief Function takes in the current mode and last mode and closes nodes and opens the new nodes needed
     *
     */
    void launch_call(std::string drive_mode_status, std::string last_mode)
    {
        if (last_mode == "Basic Drive Mode")
        {
            system("killall teleop_node");
        }
        if (drive_mode_status == "Test Mode")
        {
            system("ros2 launch bob_test test.launch.py &");
        }
        if (drive_mode_status == "Basic Drive Mode")
        {
            system("ros2 launch teleop_twist_joy teleop.launch.py &");
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

        //! Subscriber to read from the joy topic to know which buttons have been pressed
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr gamepad_subscriber;

        //! Subscriber to read from the drive_mode_status topic to know when the drive mode has changed
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscriber;

        //! Publisher to publish to the drive_mode_status topic after the driving mode button is pressed
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher;
    };



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveMode>());
    rclcpp::shutdown();
    return 0;
}