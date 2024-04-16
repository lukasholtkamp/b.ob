
#include <iostream>
#include <fmt/core.h> // For improved string formatting
#include <chrono>
#include <thread>

#include <signal.h>

#include "MotorEncoder.hpp"
#include "MotorEncoder.cpp"

void printStatus(const ENC::Encoder& left, const ENC::Encoder& right);


bool isRunning = false;
void signalHandler(int signal)
{
    std::cout << "Received signal. Shutting down." << std::endl;
    isRunning = false;
}

void callbackleft(int way){
    static int leftpos=0;
    leftpos+=way;
    // std::cout << fmt::format("Motors pos on left: {} ", leftpos) << std::endl;
}

void callbackright(int way){
    static int rightpos=0;
    rightpos+=way;
    // std::cout << fmt::format("Motors pos on rightft: {} ", rightpos) << std::endl;
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

    std::cout << fmt::format("Configuring Left Encoder on GPIO {}... ", LEFT_ENCODER_PIN);
    ENC::Encoder leftEncoder(LEFT_ENCODER_PIN,callbackleft);
    std::cout << "SUCCESS" << std::endl;

    std::cout << fmt::format("Configuring Right Motor on GPIO {}... ", RIGHT_ENCODER_PIN);
    ENC::Encoder rightEncoder(RIGHT_ENCODER_PIN,callbackright);    
    std::cout << "SUCCESS" << std::endl;

    isRunning = true;
    
    while (isRunning)
    {   
        printStatus(leftEncoder, rightEncoder);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        system("clear");
    }

    std::cout << "Cleaning up resources" << std::endl << std::flush;

    return 0;    
}

void printStatus(const ENC::Encoder& leftEncoder, const ENC::Encoder& rightEncoder)
{
    std::cout << fmt::format("Motors speed on left: {} and right: {} ", leftEncoder.getMotorSpeed(), rightEncoder.getMotorSpeed()) << std::endl;
}
