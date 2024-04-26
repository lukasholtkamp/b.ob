#include <iostream>
#include <fmt/core.h> // For improved string formatting
#include <chrono>
#include <thread>

#include <signal.h>

#include "Wheel.hpp"
#include "Wheel.cpp"

#include "MotorDriver.hpp"
#include "MotorDriver.cpp"

#include "MotorEncoder.hpp"
#include "MotorEncoder.cpp"

#include "MotorAlarm.hpp"
#include "MotorAlarm.cpp"

void printStatus(const WH::Wheel& left, const WH::Wheel& right);

WH::Wheel leftWheel;
WH::Wheel rightWheel;

bool isRunning = false;
void signalHandler(int signal)
{
    (void)signal;
    std::cout << "Received signal. Shutting down." << std::endl;
    isRunning = false;
}

// Left wheel callback function
void left_wheel_pulse(int tick)
{   
    leftWheel.update();
    if(leftWheel.Motor.getDirection() == "FORWARD")
        leftWheel.encoder_ticks+=tick;
    else if(leftWheel.Motor.getDirection() == "BACKWARD")
        leftWheel.encoder_ticks-=tick;
}

// Right wheel callback function
void right_wheel_pulse(int tick)
{
    rightWheel.update();
    if(rightWheel.Motor.getDirection() == "FORWARD")
        rightWheel.encoder_ticks+=tick;
    else if(rightWheel.Motor.getDirection() == "BACKWARD")
        rightWheel.encoder_ticks-=tick;
}

int main()
{
    int gpioResult = 0;
    
    // Print the pigpio library version
    std::cout << "Using pigpio version" << gpioVersion() << std::endl;
    std::cout << "Running on " << fmt::format("{:x}", gpioHardwareRevision()) << std::endl;

    // Disable built-in pigpio signal handling
    // Must be called before gpioInitialise()
    int cfg = gpioCfgGetInternals();
    cfg |= PI_CFG_NOSIGHANDLER;
    gpioCfgSetInternals(cfg);

    signal(SIGINT, signalHandler);

    // Initialize the pigpio library
    std::cout << "Initializing pigpio... ";
    gpioResult = gpioInitialise();
    if (gpioResult == PI_INIT_FAILED)
    {
        std::cout << "ERROR" << std::endl;
        std::cout << "Error value = " << gpioResult << std::endl;
        return -1;    
    }
    std::cout << "SUCCESS" << std::endl;

    signal(SIGINT, signalHandler);

    std::cout << fmt::format("Configuring Left & Right Wheel");

    MD::Motor leftMotor(LEFT_DIRECTION_PIN, LEFT_PWM_PIN, MAX_SPEED,CCW);
    ENC::Encoder leftEncoder(LEFT_ENCODER_PIN,left_wheel_pulse);
    ALM::Alarm leftAlarm(LEFT_ALARM_PIN);

    leftWheel.setup("left_wheel",15*6,0.08255,10,leftMotor, leftEncoder, leftAlarm);

    MD::Motor rightMotor(RIGHT_DIRECTION_PIN, RIGHT_PWM_PIN, MAX_SPEED,CCW);
    ENC::Encoder rightEncoder(RIGHT_ENCODER_PIN,right_wheel_pulse);
    ALM::Alarm rightAlarm(RIGHT_ALARM_PIN);

    rightWheel.setup("right_wheel",15*6,0.08255,10,rightMotor, rightEncoder, rightAlarm);

    std::cout << "SUCCESS" << std::endl;

    isRunning = true;
    
    while (isRunning)
    {   
        int i;
        for (i=0; i<10; i++) {
            leftWheel.set_speed(double(i)/10);
            rightWheel.set_speed(double(i)/10);
            leftWheel.update();
            rightWheel.update();
            printStatus(leftWheel,rightWheel);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            system("clear");
        }
        for (i=10; i!=0; i--) {
            leftWheel.set_speed(double(i)/10);
            rightWheel.set_speed(double(i)/10);
            leftWheel.update();
            rightWheel.update();
            printStatus(leftWheel,rightWheel);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            system("clear");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        system("clear");
    }

    leftWheel.Motor.stop();
    std::cout << "Cleaning up resources" << std::endl << std::flush;

    return 0;
}

void printStatus(const WH::Wheel& pleftWheel,const WH::Wheel& prightWheel)
{
    std::cout << fmt::format("Wheel command speed in m/s on left: {:.2f} and right: {:.2f}", pleftWheel.command,prightWheel.command) << std::endl;
    std::cout << fmt::format("Motors alarm state on left: {} and right: {}", pleftWheel.Alarm.getState(),prightWheel.Alarm.getState()) << std::endl;
    std::cout << fmt::format("Wheel speed in RPM on left: {:.2f} and right: {:.2f}", pleftWheel.Encoder.getMotorSpeed(),prightWheel.Encoder.getMotorSpeed()) << std::endl;
    std::cout << fmt::format("Wheel speed in m/s on left: {:.2f} and right: {:.2f}", pleftWheel.velocity, prightWheel.velocity) << std::endl;
}