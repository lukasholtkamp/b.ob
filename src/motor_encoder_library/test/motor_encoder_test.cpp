
#include <iostream>
#include <fmt/core.h> // For improved string formatting
#include <chrono>
#include <thread>

#include <signal.h>

#include "MotorEncoder.hpp"
#include "MotorEncoder.cpp"

#include "MotorAlarm.hpp"
#include "MotorAlarm.cpp"

void printStatus(const ENC::Encoder& left, const ENC::Encoder& right);


bool isRunning = false;
void signalHandler(int signal)
{
    std::cout << "Received signal. Shutting down." << std::endl;
    isRunning = false;
}

// temporary callback function
void callbackleft(int way){
}

// temporary callback function
void callbackright(int way){
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

     // Setting up encoders and alarms

    std::cout << fmt::format("Configuring Left Encoder on GPIO {}... ", LEFT_ENCODER_PIN);
    ENC::Encoder leftEncoder(LEFT_ENCODER_PIN,callbackleft);
    ALM::Alarm leftAlarm(LEFT_ALARM_PIN);
    std::cout << "SUCCESS" << std::endl;

    std::cout << fmt::format("Configuring Right Motor on GPIO {}... ", RIGHT_ENCODER_PIN);
    ENC::Encoder rightEncoder(RIGHT_ENCODER_PIN,callbackright);
    ALM::Alarm rightAlarm(RIGHT_ALARM_PIN);    
    std::cout << "SUCCESS" << std::endl;

    isRunning = true;

    //read motors and display the state and rpm of each left and right 
    while (isRunning)
    {   
        
        std::cout << fmt::format("Motors alarm state on left: {} and right: {} ", leftAlarm.getState(),rightAlarm.getState()) << std::endl;
        printStatus(leftEncoder, rightEncoder);
       
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        system("clear");
    }
    
    std::cout << "Cleaning up resources" << std::endl << std::flush;

    return 0;    
}

void printStatus(const ENC::Encoder& leftEncoder, const ENC::Encoder& rightEncoder)
{
    std::cout << fmt::format("Motors RPM on left: {:.2f} and right: {:.2f} ", leftEncoder.getMotorSpeed(), rightEncoder.getMotorSpeed()) << std::endl;
}
