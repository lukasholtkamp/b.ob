#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <thread>
#include <wiringPi.h>
#include <iostream>
#include <chrono>
#define LEFT_WHEEL_SPEED_PIN    3       // 3 as wiiringPi pin  15 in Raspberry pi pins ,used to detect pulses comes from left wheel speed pin
#define RIGHT_WHEEL_SPEED_PIN   4       // 4 as wiiringPi pin  16 in Raspberry pi pins ,used to detect pulses comes from right wheel speed pin
#define FREQUENCY_DIVISOR       190000  // Frequency divisor (for 6 times frequency processing)

volatile double pulseCounter_left = 0;
volatile double pulseCounter_right = 0;

void handlePulse_LeftWheel() {
        pulseCounter_left++;
}

void thread_RightWheel() {
        while (ros::ok()) {
                if(digitalRead(RIGHT_WHEEL_SPEED_PIN) == 1 || digitalRead(RIGHT_WHEEL_SPEED_PIN) == -1) {
                        pulseCounter_right++;
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
        }
}

int main(int argc, char **argv) {
        ros::init(argc, argv, "motor_speed_publisher");
        ros::NodeHandle nh;
        ros::Publisher speed_pub1 = nh.advertise<std_msgs::Float32>("Motor_Speed_Node1", 5);
        ros::Publisher speed_pub2 = nh.advertise<std_msgs::Float32>("Motor_Speed_Node2", 5);

        wiringPiSetup();
        if (wiringPiSetup() == -1) {
                std::cerr << "Failed to initialize WiringPi." << std::endl;
                return 1;
        }
        pinMode(LEFT_WHEEL_SPEED_PIN, INPUT);
        pinMode(RIGHT_WHEEL_SPEED_PIN, INPUT);

        pullUpDnControl(RIGHT_WHEEL_SPEED_PIN, PUD_UP);

        wiringPiISR(LEFT_WHEEL_SPEED_PIN ,INT_EDGE_BOTH, &handlePulse_LeftWheel);
        std::thread right_wheel_thread(thread_RightWheel);
        ros::Rate loop_rate(5); // Publish at 5 Hz

        while (ros::ok()) {

        std_msgs::Float32 speed_msg1;
                speed_msg1.data = pulseCounter_left / FREQUENCY_DIVISOR;

                std_msgs::Float32 speed_msg2;
                speed_msg2.data = pulseCounter_right / 1000;

                speed_pub1.publish(speed_msg1);
                speed_pub2.publish(speed_msg2);

                pulseCounter_right = 0;
                pulseCounter_left = 0;

                ros::spinOnce();
                loop_rate.sleep();
        }
        right_wheel_thread.join();
        return 0;
}

