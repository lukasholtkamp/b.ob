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

void printStatus(const WH::Wheel& left);

WH::Wheel leftWheel;
WH::Wheel righttWheel;

bool isRunning = false;
void signalHandler(int signal)
{
    std::cout << "Received signal. Shutting down." << std::endl;
    isRunning = false;
}

// Left wheel callback function
void left_wheel_pulse(int tick)
{   
    if(leftWheel.Motor.getDirection() == "FORWARD")
        leftWheel.encoder_ticks++;
    else if(leftWheel.Motor.getDirection() == "BACKWARD")
        leftWheel.encoder_ticks--;
}

// Right wheel callback function
// void right_wheel_pulse()
// {

// }

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

    std::cout << fmt::format("Configuring Left Wheel");

    MD::Motor leftMotor(LEFT_DIRECTION_PIN, LEFT_PWM_PIN, MAX_SPEED,CCW);
    ENC::Encoder leftEncoder(LEFT_ENCODER_PIN,left_wheel_pulse);
    ALM::Alarm leftAlarm(LEFT_ALARM_PIN);

    leftWheel.setup("left_wheel",15*6,0.08255,10,leftMotor, leftEncoder, leftAlarm);

    std::cout << "SUCCESS" << std::endl;

    isRunning = true;

    leftWheel.set_speed(50);
    
    while (isRunning)
    {   
        leftWheel.update();
        printStatus(leftWheel);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        system("clear");
    }

    leftWheel.Motor.stop();
    std::cout << "Cleaning up resources" << std::endl << std::flush;

    return 0;    
}

void printStatus(const WH::Wheel& leftWheel)
{
    // std::cout << fmt::format("Wheel command speed in m/s on left: {:.2f} ", leftWheel.command) << std::endl;
    std::cout << fmt::format("Wheel speed in m/s on left: {:.2f} ", leftWheel.Encoder.getMotorSpeed()) << std::endl;
}