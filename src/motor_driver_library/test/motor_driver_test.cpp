#include <iostream>
#include <fmt/core.h>
#include <chrono>
#include <thread>

#include <signal.h>

#include "MotorDriver.hpp"
#include "MotorDriver.cpp"

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

    // Setting up motors

    std::cout << fmt::format("Configuring Left Motor on GPIO {}, & {}... ", LEFT_DIRECTION_PIN, LEFT_PWM_PIN);
    MD::Motor leftMotor(LEFT_DIRECTION_PIN, LEFT_PWM_PIN, MAX_SPEED,CCW);
    std::cout << "SUCCESS" << std::endl;

    std::cout << fmt::format("Configuring Right Motor on GPIO {}, & {}... ", RIGHT_DIRECTION_PIN, RIGHT_PWM_PIN);
    MD::Motor rightMotor(RIGHT_DIRECTION_PIN, RIGHT_PWM_PIN, MAX_SPEED,CW);    
    std::cout << "SUCCESS" << std::endl;

    isRunning = true;

    // Run motor from 0 to 100% of max speed both foward and backward
    while (isRunning)
    {
    
        int i;
        for (i=0; i<100; i++) {
            leftMotor.setSpeed(i);
            rightMotor.setSpeed(i);
            printStatus(leftMotor, rightMotor);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            system("clear");
        }
        for (i=100; i!=0; i--) {
            leftMotor.setSpeed(i);
            rightMotor.setSpeed(i);
            printStatus(leftMotor, rightMotor);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            system("clear");
        }

        for (i=0; i > -100; i--) {
            leftMotor.setSpeed(i);
            rightMotor.setSpeed(i);
            printStatus(leftMotor, rightMotor);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            system("clear");
        }
        for (i=-100; i!=0; i++) {
            leftMotor.setSpeed(i);
            rightMotor.setSpeed(i);
            printStatus(leftMotor, rightMotor);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            system("clear");
        }

    
    }
    
    // stop motors
    leftMotor.stop();
    rightMotor.stop();
    system("clear");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    printStatus(leftMotor, rightMotor);
    std::cout << "Cleaning up resources" << std::endl << std::flush;

    return 0;  

}

void printStatus(const MD::Motor& leftMotor, const MD::Motor& rightMotor)
{
    std::cout << fmt::format("Motors set to left: {} @ {} % and right: {} @ {} %", leftMotor.getDirection(), leftMotor.getSpeed(), rightMotor.getDirection(), rightMotor.getSpeed()) << std::endl;
}
