#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <wiringPi.h>

const int inputPin = 3; // Use the appropriate GPIO pin number
const int frequencyDivisor = 12; // Frequency divisor (for 6 times frequency processing)

volatile int pulseCount = 0;

void handlePulse() {
    pulseCount++;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "motor_speed_publisher");
    ros::NodeHandle nh;
    ros::Publisher speed_pub = nh.advertise<std_msgs::Int32>("motor_speed", 10);

    if (wiringPiSetupGpio() == -1) {
        ROS_ERROR("Error initializing WiringPi.");
        return 1;
    }

    pinMode(inputPin, INPUT);
    //pullUpDnControl(inputPin, PUD_UP);
    wiringPiISR(inputPin, INT_EDGE_RISING, &handlePulse);

    ros::Rate loop_rate(1); // Publish at 1 Hz

    while (ros::ok()) {
        std_msgs::Int32 speed_msg;
        speed_msg.data = pulseCount / frequencyDivisor;

        speed_pub.publish(speed_msg);
        pulseCount = 0;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
