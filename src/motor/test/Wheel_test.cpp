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

// Initialize pulse counters
int left_wheel_pulse_count = 0;
int right_wheel_pulse_count = 0;

// Initialize wheel directions
std::string left_wheel_direction = "IDLE";
std::string right_wheel_direction = "IDLE";

void printStatus(const WH::Wheel& left, const WH::Wheel& right);

bool isRunning = false;
void signalHandler(int signal)
{
    (void)signal;
    std::cout << "Received signal. Shutting down." << std::endl;
    isRunning = false;
}

void read_encoder_values(int *left_encoder_value, int *right_encoder_value)
{
    *left_encoder_value = left_wheel_pulse_count;
    *right_encoder_value = right_wheel_pulse_count;
}

void set_motor_direction(std::string left_motor_dir, std::string right_motor_dir)
{
    left_wheel_direction  = left_motor_dir;
    right_wheel_direction  = right_motor_dir;
}


// Left wheel callback function
void left_wheel_pulse(int tick)
{   
    if(left_wheel_direction == "FORWARD")
        left_wheel_pulse_count+=tick;
    else if(left_wheel_direction== "BACKWARD")
        left_wheel_pulse_count-=tick;
}

// Right wheel callback function
void right_wheel_pulse(int tick)
{
    if(right_wheel_direction == "FORWARD")
        right_wheel_pulse_count+=tick;
    else if(right_wheel_direction == "BACKWARD")
        right_wheel_pulse_count-=tick;
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

    MD::Motor leftMotor(LEFT_DIRECTION_PIN, LEFT_PWM_PIN, 15,CCW);
    ENC::Encoder leftEncoder(LEFT_ENCODER_PIN,left_wheel_pulse);
    ALM::Alarm leftAlarm(LEFT_ALARM_PIN);

    WH::Wheel leftWheel("left_wheel",69,0.08255,leftMotor, leftEncoder, leftAlarm);

    MD::Motor rightMotor(RIGHT_DIRECTION_PIN, RIGHT_PWM_PIN, 15,CW);
    ENC::Encoder rightEncoder(RIGHT_ENCODER_PIN,right_wheel_pulse);
    ALM::Alarm rightAlarm(RIGHT_ALARM_PIN);

    WH::Wheel rightWheel("right_wheel",69,0.08255,rightMotor, rightEncoder, rightAlarm);

    std::cout << "SUCCESS" << std::endl;

    isRunning = true;
    
    while (isRunning)
    {   
        rightWheel.set_speed(255);
        leftWheel.set_speed(255);
        set_motor_direction(leftWheel.Motor.getDirection(),rightWheel.Motor.getDirection());
        read_encoder_values(&leftWheel.encoder_ticks, &rightWheel.encoder_ticks);
        rightWheel.update();
        printStatus(leftWheel,rightWheel);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        

        if(fabs(rightWheel.old_position)>=1){
            isRunning = false;
        }
        else{
            system("clear");
        }

        

    }

    leftWheel.Motor.stop();
    rightWheel.Motor.stop();
    std::cout << "Cleaning up resources" << std::endl << std::flush;

    return 0;
}

void printStatus(const WH::Wheel& pleftWheel,const WH::Wheel& prightWheel)
{
    std::cout << fmt::format("Wheel command speed in m/s on left: {:.2f} and right: {:.2f}", pleftWheel.command,prightWheel.command) << std::endl;
    std::cout << fmt::format("Wheel direction on left: {} and right: {}", pleftWheel.Motor.getDirection(),prightWheel.Motor.getDirection()) << std::endl;
    std::cout << fmt::format("Wheel position on left: {} and right: {}", pleftWheel.position,prightWheel.position) << std::endl;
    std::cout << fmt::format("Motors alarm state on left: {} and right: {}", pleftWheel.Alarm.getState(),prightWheel.Alarm.getState()) << std::endl;
    std::cout << fmt::format("Wheel speed in RPM on left: {:.2f} and right: {:.2f}", pleftWheel.Encoder.getMotorSpeed(),prightWheel.Encoder.getMotorSpeed()) << std::endl;
    std::cout << fmt::format("Wheel speed in m/s on left: {:.2f} and right: {:.2f}", pleftWheel.velocity, prightWheel.velocity) << std::endl;
}