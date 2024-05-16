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

std::string find_button(std::vector<int> buttons);
void launch_call(std::string mode, std::string last_mode);
// std::string find_axes(std::vector<float> axes,float size);
// std::string publish

class DriveModeSubscriber : public rclcpp::Node
{
public:
    DriveModeSubscriber()
        : Node("drive_selection")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&DriveModeSubscriber::topic_callback, this, _1));

        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "drive_mode_status", 10, std::bind(&DriveModeSubscriber::status_callback, this, _1));

        publisher_ = this->create_publisher<std_msgs::msg::String>("drive_mode_status", 10);
    }

    std::string last_mode = "IDLE";

private:
    void topic_callback(const sensor_msgs::msg::Joy &msg)
    {
        // // std::string axes=find_axes(msg.axes,msg.axes.size());

        // if (button != "")
        // {
        //     std::cout << button << std::endl;
        //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
        //     system("clear");
        //     std::cout << button << std::endl;
        // }
        /*
        if(axes!=""){
            std::cout << axes << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            system("clear");
            std::cout << axes << std::endl ;
        }
        */
        auto drive_mode_status = std_msgs::msg::String();

        if (find_button(msg.buttons) == "")
        {
            drive_mode_status.data = last_mode;
        }
        else
        {
            drive_mode_status.data = find_button(msg.buttons);
        }

        publisher_->publish(drive_mode_status);
    }

    void status_callback(const std_msgs::msg::String &msg)
    {
        if(last_mode != msg.data)
        {
            std::cout << "Driving Mode: " << msg.data << std::endl ;

            launch_call(msg.data, last_mode);
            last_mode = msg.data;
            
        }
        
        
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

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
        return "";
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
        // Button Menu
        // system("shutdown now");
        return "Shutdown Button";
    }
    else
    {
        return "";
    }
}

/*
std::string find_axes(std::vector<float> axes, int size)
{
    int pressed = -1;

    for (int i = 0; i < size; i++)
    {

        if (buttons[i] > 0)
        {
            pressed = i;
            break;
        }
    }

    if (pressed == 0)
    {
        //  Left TS vertical Axes
        return "Left Thumbstick vertikal Axes!";
    }
    if (pressed == 1)
    {
        // Left TS linear Axes
        system("ĵoy_linux_node");
        return "Left Thumbstick linear Axes!";
    }
    if (pressed == 2)
    {
        // Right TS vertical Axes
        return "Right Thumbstick vertikal Axes!";
    }
    if (pressed == 3)
    {
        // Right TS linear Axes
        return "Right Thumbstick linear Axes";
    }
    if (pressed == 4)
    {
        // Trigger RT
        return "Right Trigger pressed!";
    }
    if (pressed == 5)
    {
        // Trigger LT
        return "Left Trigger pressed!!";
    }
    if (pressed == 6)
    {
        // D-pad vertical
        return "D-pad vertical Axes!";
    }
    if (pressed == 7)
    {
        // D-pad linear
        return "D-pad linear Axes!";
    }
    else
    {
        return "";
    }
}
*/

void launch_call(std::string drive_mode_status, std::string last_mode)
{
    if(last_mode=="Basic Drive Mode"){
        system("killall teleop_node");
    }
    if (drive_mode_status=="Basic Drive Mode")
    {
        system("ros2 launch teleop_twist_joy teleop-launch.py &");
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveModeSubscriber>());
    rclcpp::shutdown();
    return 0;
}
