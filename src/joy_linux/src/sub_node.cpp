#include <memory>
#include <string>
#include <iostream>
#include <stdint.h> //<-- Used to define the int32
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>

using std::placeholders::_1;

std::string find_button(std::vector<int> buttons,int size);

class DriveModeSubscriber : public rclcpp::Node
{
    public:
    DriveModeSubscriber()
    : Node("drive_mode")
    {
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&DriveModeSubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const sensor_msgs::msg::Joy & msg) const
        {
        std::string button=find_button(msg.buttons,msg.buttons.size());

        if(button!=""){
            std::cout << button << std::endl ;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            system("clear");
            std::cout << button << std::endl ;
        }
        
        }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

std::string find_button(std::vector<int> buttons,int size){
    int pressed=-1;

    for (int i = 0; i < size; i++){

        if(buttons[i]>0)
        {
            pressed=i;
            break;
        }
    }

        if (pressed==0)
        {
            //Button A
            return "Assisted Drive Mode Enabled";
        }
        if (pressed==1)
        {
            //Button B
            return "Emergency Stop";
        }
        if (pressed==4)
        {
            //Button Y
            return "";
        }
        if (pressed==3)
        {
            //Button X
            return "Basic Drive Mode Enabled";
        }
        else
        {
            return "";
        }

}

int main(int argc, char * argv[])
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<DriveModeSubscriber>());
rclcpp::shutdown();
return 0;
}
