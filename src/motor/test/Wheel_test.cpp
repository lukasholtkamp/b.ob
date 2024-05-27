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

#define LEFTWHEEL_PULSES_PER_REV 98
#define RIGHTWHEEL_PULSES_PER_REV 98

#define WHEEL_RADIUS 0.084

// Initialize pulse counters
int left_wheel_pulse_count = 0;
int right_wheel_pulse_count = 0;

double left_input_sig = 0;
double right_input_sig = 0;

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

void read_motor_signal(double linsig, double rinsig)
{
    left_input_sig  = linsig;
    right_input_sig  = rinsig;
}


// Left wheel callback function
void left_wheel_pulse(int tick)
{   
    if(left_wheel_direction == "FORWARD"){
        left_wheel_pulse_count+=tick;
    }
    else if(left_wheel_direction== "BACKWARD"){
        left_wheel_pulse_count-=tick;
    }
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

    MD::Motor leftMotor(LEFT_DIRECTION_PIN, LEFT_PWM_PIN, 100,CCW);
    ENC::Encoder leftEncoder(LEFT_ENCODER_PIN,left_wheel_pulse, LEFTWHEEL_PULSES_PER_REV);
    ALM::Alarm leftAlarm(LEFT_ALARM_PIN);

    WH::Wheel leftWheel("left_wheel",LEFTWHEEL_PULSES_PER_REV,WHEEL_RADIUS,leftMotor, leftEncoder, leftAlarm);

    leftWheel.set_PID(75.0,175.0,3.0);  

    MD::Motor rightMotor(RIGHT_DIRECTION_PIN, RIGHT_PWM_PIN,100,CW);
    ENC::Encoder rightEncoder(RIGHT_ENCODER_PIN,right_wheel_pulse, RIGHTWHEEL_PULSES_PER_REV);
    ALM::Alarm rightAlarm(RIGHT_ALARM_PIN);

    WH::Wheel rightWheel("right_wheel",RIGHTWHEEL_PULSES_PER_REV,WHEEL_RADIUS,rightMotor, rightEncoder, rightAlarm);

    rightWheel.set_PID(75.0,175.0,3.0); 

    std::cout << "SUCCESS" << std::endl;

    isRunning = true;

    double speed = 20;
    rightWheel.set_speed(speed);
    leftWheel.set_speed(speed);
    
    while (isRunning)
    {   
        
        set_motor_direction(leftWheel.Motor.getDirection(),rightWheel.Motor.getDirection());
        read_encoder_values(&leftWheel.encoder_ticks, &rightWheel.encoder_ticks);
        // read_motor_signal(leftWheel.u,rightWheel.u);

        // rightWheel.update();
        // leftWheel.update();
        
        printStatus(leftWheel,rightWheel);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // system("clear");

        if(rightWheel.encoder_ticks>=42){
            rightWheel.set_speed(0);
        }
        if(leftWheel.encoder_ticks>=44){
            leftWheel.set_speed(0);
        }
        if(rightWheel.encoder_ticks>=42 && leftWheel.encoder_ticks>=45){
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
    std::cout << fmt::format("Wheel pulses on left: {} and right: {}", pleftWheel.encoder_ticks,prightWheel.encoder_ticks) << std::endl;
    // std::cout << fmt::format("Wheel command speed in m/s on left: {:.2f} and right: {:.2f}", pleftWheel.command,prightWheel.command) << std::endl;
    // std::cout << fmt::format("Wheel speed in m/s on left: {:.2f} and right: {:.2f}", pleftWheel.velocity, prightWheel.velocity) << std::endl;
    // std::cout << fmt::format("Wheel speed signal on left: {:.2f} and right: {:.2f}", pleftWheel.u,prightWheel.u) << std::endl;
    // std::cout << fmt::format("Wheel direction on left: {} and right: {}", pleftWheel.Motor.getDirection(),prightWheel.Motor.getDirection()) << std::endl;
    // std::cout << fmt::format("Wheel position in m on left: {} and right: {}", pleftWheel.position,prightWheel.position) << std::endl;
    // std::cout << fmt::format("Motors alarm state on left: {} and right: {}", pleftWheel.Alarm.getState(),prightWheel.Alarm.getState()) << std::endl;
    // std::cout << fmt::format("Wheel speed in RPM on left: {:.2f} and right: {:.2f}", pleftWheel.Encoder.getMotorSpeed(),prightWheel.Encoder.getMotorSpeed()) << std::endl;
}