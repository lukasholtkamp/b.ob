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

// Left wheel callback function
void left_wheel_pulse(int tick)
{   
    if(Wheel.Motor.getDirection() == "FORWARD")
        Wheel.encoder_ticks++;
    else if(Wheel.Motor.getDirection() == "BACKWARD")
        Wheel.encoder_ticks--;
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

    Wheel("left_wheel",15*6,0.08255,10,leftMotor, leftEncoder, leftAlarm);

    std::cout << "SUCCESS" << std::endl;

    isRunning = true;
    
    while (isRunning)
    {   
        Wheel.set_speed(2);
        Wheel.update();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        system("clear");
    }

    std::cout << "Cleaning up resources" << std::endl << std::flush;

    return 0;    
}