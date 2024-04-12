#include <iostream>
#include <fmt/core.h> // For improved string formatting
#include <chrono>
#include <thread>

#include <signal.h>

#include "MotorDriver.hpp"
#include "MotorDriver.cpp"
#include "RunMotor.hpp"

void printStatus(const MD::Motor& left, const MD::Motor& right);


bool isRunning = false;
void signalHandler(int signal)
{
    std::cout << "Received signal. Shutting down." << std::endl;
    isRunning = false;
}

int main()
{
    int gpioResult = 0;
    
    // Print the pigpio library version
    std::cout << "Using pigpio version " << gpioVersion() << std::endl;
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

    std::cout << fmt::format("Configuring Left Motor on GPIO {}, & {}... ", LEFT_DIRECTION_PIN, LEFT_PWM_PIN);
    MD::Motor leftMotor(LEFT_DIRECTION_PIN, LEFT_PWM_PIN, MIN_SPEED, MAX_SPEED,CCW);
    std::cout << "SUCCESS" << std::endl;

    std::cout << fmt::format("Configuring Right Motor on GPIO {}, & {}... ", RIGHT_DIRECTION_PIN, RIGHT_PWM_PIN);
    MD::Motor rightMotor(RIGHT_DIRECTION_PIN, RIGHT_PWM_PIN, MIN_SPEED, MAX_SPEED,CW);    
    std::cout << "SUCCESS" << std::endl;

    isRunning = true;
    
    while (isRunning)
    {
    
        leftMotor.setSpeed(50.0);
        rightMotor.setSpeed(50.0);

        printStatus(leftMotor, rightMotor);

        std::this_thread::sleep_for(std::chrono::seconds(3));

        leftMotor.setSpeed(0.0);
        rightMotor.setSpeed(0.0);

        printStatus(leftMotor, rightMotor);

        std::this_thread::sleep_for(std::chrono::seconds(3));

        leftMotor.setSpeed(-50.0);
        rightMotor.setSpeed(-50.0);

        printStatus(leftMotor, rightMotor);

        std::this_thread::sleep_for(std::chrono::seconds(3));

        leftMotor.setSpeed(0.0);
        rightMotor.setSpeed(0.0);

        printStatus(leftMotor, rightMotor);

        std::this_thread::sleep_for(std::chrono::seconds(3));
    }

    std::cout << "Cleaning up resources" << std::endl << std::flush;

    return 0;    
}

void printStatus(const MD::Motor& leftMotor, const MD::Motor& rightMotor)
{
    std::cout << fmt::format("Motors set to left: {} @ {} % and right: {} @ {} %", leftMotor.getDirection(), leftMotor.getSpeed(), rightMotor.getDirection(), rightMotor.getSpeed()) << std::endl;
}
