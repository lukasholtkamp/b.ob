#include "MotorDriver.h"
#include "RunMotor.h"
#include "XBoxController.h"
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <chrono>
#include <thread>

using namespace std;
using namespace std::this_thread;
using namespace std::chrono;

/********************global Variables********************/
//---Values for Gyroscope---
const int ACCEL_SCALE = 16384.0;                //define the scale of the acceleration to give readable values on m/s2
const float GYROSCOPE_SCALE = 7.07*131;         //define the scale of gyroscope to give readable values on °/s
const float Kp = 2.0;                           //Propotional gain
const float Ki = 0.0;                           //integral gain
const float Kd = 0.00;                          //Derivative gain
#define WHEEL_DIAMETER          17.5    //define the diameter of the wheels 
double gz;
const int inputPin = 3;
volatile int pulseCount =0;
//---Values for autonomous driving---
float baseSpeed = 20;                           // base speed for the wheels
int position = -1;
int auto_check = 0;
sensor_msgs::Imu latest_imu_data;
float current_heading {0};
float desired_heading {};
double traveled_distance {};  
float error {};
float prev_error {};
float accumulated_error {};                     //for integral accumulation
float steering {};
//---Values Setting the Motor---
float leftWheelSpeed {};
float rightWheelSpeed {};
float newLeftWheelSpeed {};
float newRightWheelSpeed {};
float angularVelocityZ {};
float linearVelocityX {};
bool LeftWheelDirection {true};
bool RightWheelDirection {true} ;
//---Values to measure time---
static ros::Time prev_time;
static ros::Time current_time;
static ros::Time prev_time_speed;
static ros::Time current_time_speed;
double dt {};
double actual_time = 0;
//---Values to use for positional Data---
bool scanFlag {true};
int movementCounter = 0;
int qrData = -1;
int prev_data = -1;
bool move_flag = true;
//---Values for Lidar---
#define RAD2DEG(x) ((x)*180./M_PI)
float RightFront=0;
float Right90Degrees=0;
float RightBack=0;
float RightDetection=0;
float Front=0;
float LeftFront=0;
float Left90Degrees=0;
float LeftBack=0;
float sideFront = 0;
bool obstacleDetected = false;
//---Web page---
std_msgs::String robot_cmd;
std_msgs::String robot_status;
/*******************************************************/
MD::GY521 gy521(0x68);                                          // an object of gyroscope class using I2c 0x68 address
enum State {
processing,
idle,
emergency_pressed
};

State state_bob = idle;
MD::Motor leftMotor(LEFT_DIRECTION_PIN, LEFT_PWM_PIN, MIN_SPEED, MAX_SPEED); //<-- The left motor object
MD::Motor rightMotor(RIGHT_DIRECTION_PIN, RIGHT_PWM_PIN, MIN_SPEED, MAX_SPEED); //<-- The right motor object
//XB::XBoxController xbox;

void MovementSetting(float linearVelocityX,float angularVelocityZ) {
         leftWheelSpeed = leftMotor.LinearAndAngularVelocities(linearVelocityX, angularVelocityZ);
         rightWheelSpeed = rightMotor.LinearAndAngularVelocities(linearVelocityX, angularVelocityZ*-1);


        leftMotor.setSpeed(leftWheelSpeed); //<-- Set the speed of the left motor
        rightMotor.setSpeed(rightWheelSpeed); //<-- Set the speed of the right motor

        leftMotor.setDirection(leftWheelSpeed, LEFT_DIRECTION_PIN); //<-- Set the direction of the left motor
        rightMotor.setDirection(-1*rightWheelSpeed, RIGHT_DIRECTION_PIN); //<-- Set the direction of the right motor

        LeftWheelDirection = leftMotor.getDirection();
        RightWheelDirection = rightMotor.getDirection(); // reversed to mirror the left motor

                newLeftWheelSpeed = leftMotor.getSpeed(); //<-- The new speed of the left wheel
                newRightWheelSpeed = rightMotor.getSpeed(); //<-- The new speed of the right wheel
        cout<<"linear Velocity: "<<linearVelocityX<<endl;
        cout<<"steering: "<<steering<<endl;

        ROS_INFO_STREAM("angular Velocity: "<<angularVelocityZ);
        softPwmWrite(LEFT_PWM_PIN,newLeftWheelSpeed); //<-- Set the speed of the left motor
        softPwmWrite(RIGHT_PWM_PIN,newRightWheelSpeed); //<-- Set the speed of the right motor


}
void rotateBob(float desiredAngle) {
        desired_heading = desiredAngle;
        if(current_heading != desired_heading) 
        {
                state_bob = processing;
                cout<<"current_heading inside if: "<<current_heading<<endl;
                linearVelocityX = 0;
                //calculate time 
                //use time to find the direction
                 ros::Time current_time = ros::Time::now();
                dt = (current_time - prev_time).toSec();
                prev_time = current_time;
                cout<<"gz: "<<gz<<endl;
                current_heading += ((gz * dt)*(180/M_PI));//+0.005;
                error = desired_heading - current_heading;
                if (std::abs(error) < 0.5) {
                        error = 0.0;
                }

                cout<<"steering before: "<<steering<<endl;
                // Calculate the control signal (steering) using the PID controller
                steering = Kp * error + Ki * accumulated_error + Kd * (error - prev_error);

                // Update the accumulated error for the integral term
                accumulated_error += error * dt;

                // Save the current error for the next iteration
                prev_error = error;
                //limit the Steering to stop it from spinning too fast
                if(steering > 10)
                {
                        steering = 10;
                }
                else if(steering < -10)
                {
                        steering = -10;
                }
                cout<<"steering after: "<<steering<<endl;
                //setting Speed for the Wheels
                MovementSetting(linearVelocityX,steering*-1);
                //set time for next iteration
                prev_time = current_time;
                if(abs(error) == 0)
                {
                        MovementSetting(0,0);
                        //counter up to get to next Movement
                        movementCounter--;

                        sleep_for(seconds(1));
                         ros::Time current_time = ros::Time::now();
                        prev_time = current_time;
                         linearVelocityX = 0;
                        error=0;
                        current_heading = 0;
                        desired_heading = 0;
                        steering = 0;
                        gz = 0;
                         dt = 0;
                        actual_time = 0;
                        latest_imu_data.angular_velocity.z = 0;
                        state_bob = idle;
                }

        }
        //dt = 0;
}
float infScale(float laserValue)
{
        if(isinf(laserValue))
                return 12;
        else
                return laserValue;
}
void moveBob(float desiredDistance)
{

        if(auto_check == 0)
        {
        state_bob = processing;
        move_flag = false;
        //set desired_heading to go straight
        desired_heading = 0;
        //set Speed to a constant to make it more simple
        linearVelocityX = baseSpeed;
        //with baseSpeed = 20 1Meter = 3.1 Seconds
        if(Front > 0.35)
        {
                //change between left and right side scan
                if(desiredDistance == 0)
                {
                        current_heading =(infScale(RightBack) - infScale(RightFront));
                        sideFront = RightFront; //((gz *dt)*(180/M_PI))-0.015;
                        cout<<"right front"<<RightFront<<endl;
                        cout<<"right back"<<RightBack<<endl;
                        cout<<"right Detection"<<RightDetection<<endl;

                }
                else if(desiredDistance == 1)
                {
                        current_heading = (infScale(LeftFront) - infScale(LeftBack));
                        sideFront = LeftFront;
                        cout<<"left front"<<LeftFront<<endl;
                        cout<<"left back"<<LeftBack<<endl;

                }
                //let it drive between 70 and 80cm away from the wall
                if(!isinf(current_heading))
                {
                        linearVelocityX = baseSpeed * (Front * Front)+3;
                        if(linearVelocityX > baseSpeed)
                                linearVelocityX = baseSpeed;
                        if((LeftFront +RightFront) > 0.9)
                        {
                        if(sideFront < 0.35)
                        {
                                if(desiredDistance == 0)
                                        error = -0.07;
                                else if(desiredDistance == 1)
                                        error =0.07;
                        }
                        else if(sideFront > 0.5)
                        {
                                linearVelocityX = baseSpeed;
                                if(desiredDistance == 0)
                                {
                                        if(RightDetection > 1)
                                                error = 0.15;
                                                //error = 0.02*(infScale(RightDetection));
                                        else if (RightDetection < 1)
                                                error = 0.07;
                                }
                                else if(desiredDistance == 1)
                                        error = -0.07;

                        }

                        else
                        {
                                error = desired_heading - current_heading;
                        }
                        }
                        else if((LeftFront +RightFront)<=0.9)
                        {
                                  if(sideFront<(LeftFront +RightFront)/2)
                        {
                                if(desiredDistance == 0)
                                        error = -0.07;
                                else if(desiredDistance == 1)
                                        error =0.07;
                        }
                        else if(sideFront>(LeftFront +RightFront)/2+0.1)
                        {
                                linearVelocityX = baseSpeed;
                                if(desiredDistance == 0)
                                {
                                        if(RightDetection > 1)
                                                error = 0.15;
                                                //error = 0.02*(infScale(RightDetection));
                                        else if (RightDetection < 1)
                                                error = 0.07;
                                }
                                else if(desiredDistance == 1)
                                        error = -0.07;

                        }
                        
                        else
                        {
                                error = desired_heading - current_heading;
                        }
                        }


                }

                cout<<"error"<<error<<endl;
                error=error*100;
                // Calculate the control signal (steering) using the PID controller
                steering = Kp * error + Ki * accumulated_error + Kd * (error - prev_error);
                // Update the accumulated error for the integral term
                accumulated_error += error * dt;

                // Save the current error for the next iteration
                prev_error = error;
                //limit the Steering to stop it from spinning too fast
                if(steering > 12)
                {
                        steering = 12;
                }
                else if(steering < -12)
                {
                        steering = -12;
                }
                //setting Speed for the Wheels
                MovementSetting(linearVelocityX,steering);
                prev_time = current_time;

        }
    else if((Front < 0.35) && (qrData == prev_data))
        {
                state_bob = idle;
                MovementSetting(0,0);
                cout<<"obstacle"<<endl;
                 cout<<"qrdata: "<<qrData<<endl;
                cout<<"prev_data: "<<prev_data<<endl;
                error=0;
                move_flag = true;
                current_heading = 0;
                desired_heading = 0;
                steering = 0;
                gz = 0;
                dt = 0;
                actual_time = 0;
                latest_imu_data.angular_velocity.z = 0;
                ros::Time current_time = ros::Time::now();
                prev_time = current_time;
                movementCounter++;

        }
        else
        {
                state_bob= idle;
                //set Motor to stop
                MovementSetting(0,0);
                cout<<"wall"<<endl;
                cout<<"qrdata: "<<qrData<<endl;
                cout<<"prev_data: "<<prev_data<<endl;
                cout<<"position: "<<position<<endl;
                //counter up to get to next Movement
                 move_flag = true;
                cout<<"moveFlag: "<<move_flag<<endl;
                //sleep_for(seconds(1));
                //cout<<"stop"<<endl;
                //cleanup values to be sure
                linearVelocityX = 0;
                error=0;
                current_heading = 0;
                desired_heading = 0;
                steering = 0;
                gz = 0;
                dt = 0;
                actual_time = 0;
                latest_imu_data.angular_velocity.z = 0;
                ros::Time current_time = ros::Time::now();
                prev_time = current_time;
                //rotateBob(-180);

        }
        }
}


void imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg) {

        //set time for first iteraion to stop data from corrupting
        if (prev_time.isZero()) {
                prev_time = msg->header.stamp;
                return;
        }
        //
        latest_imu_data = *msg;
        //get gz and put it on the right scale
        gz = latest_imu_data.angular_velocity.z/GYROSCOPE_SCALE;
        if(std::abs(gz) < 0.01) {
                gz = 0;
        }

}

void qrCallback(const std_msgs::Int32::ConstPtr& msg)
{
  if(auto_check == 0)
  {
        if (move_flag == true) {
                //if(prev_data != msg->data) {
                        MovementSetting(0,0);
                        movementCounter = 0;
                        current_time = ros::Time(0);
                        prev_time = ros::Time(0);
                        qrData = msg->data;
                //}
        }
        //qrData = msg->data;
        position = qrData;
        //position = prev_data;
        prev_data = msg->data;
        ROS_INFO_STREAM("position in Qr: "<<position);
  }
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
//int count = scan->scan_time / scan->time_increment;
  if(auto_check == 0)
  {
    RightFront=scan->ranges[318];
    RightDetection = scan->ranges[477];
   // Right90Degrees=scan->ranges[286];
    RightBack=scan->ranges[254];
    Front=scan->ranges[571];
   // Left90Degrees=scan->ranges[860];
    LeftFront=scan->ranges[836];
    LeftBack=scan->ranges[891];
    switch (position) {
    //Case scaning qrcode A
    case 0:
        if(movementCounter == 0)
                moveBob(0);
        if(movementCounter == 1)
                rotateBob(-90);
        break;
   case 1:
        if(movementCounter == 0)
                rotateBob(-180);
        if(movementCounter == 1)
                moveBob(1);

        default:
                //MovementSetting(0,0);
                break;
        }
  }





  /*  for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        ROS_INFO_STREAM("count: "<<i<<" "<< degree<<" " << scan->ranges[i]);
    }*/
}

void autoCallback(const std_msgs::Int32::ConstPtr& auto_msg)
{
        auto_check = auto_msg->data;
}
void messageCallback(const geometry_msgs::Twist& cmd_vel) {
        // Getting Velocity from XboxController.cpp and calculating Speed and direction
        if(auto_check == 1)
        {
                position = -1;
                linearVelocityX = cmd_vel.linear.x * VELOCITY_MULTIPLIER;
                angularVelocityZ = cmd_vel.angular.z * VELOCITY_MULTIPLIER;
                if(linearVelocityX > 0)
                        MovementSetting(linearVelocityX,angularVelocityZ * -1);
                else
                        MovementSetting(linearVelocityX,angularVelocityZ);
        }
}

void motorSpeedCallback(const std_msgs::Int32::ConstPtr& msg) {
    double motorSpeed = WHEEL_DIAMETER*M_PI*msg->data; //Get the speed in m/s
    ros::Time current_time_speed = ros::Time::now();
    ros::Duration elapsedTime = current_time_speed - prev_time_speed;

    // Calculate the traveled distance
    double distanceTraveled = motorSpeed * elapsedTime.toSec();

    // Accumulate the traveled distance
    traveled_distance += distanceTraveled;
    // Update the previous time
    prev_time_speed = current_time_speed;
    std::cout << "Received motor speed: "<< motorSpeed << "m/s" << std::endl;
    std::cout << "Traveled distance: "<< traveled_distance << "meters" << std::endl;
}

void signCallback(const std_msgs::String::ConstPtr& sign_msg) {
    string sign = sign_msg->data;
    
    if (sign == "Stop") {
        // Stop the robot for 3 seconds
        MovementSetting(0, 0);
        sleep_for(seconds(3));
    } else if (sign == "ahead") {
        // Continue driving at a constant speed
        MovementSetting(baseSpeed, 0);
    } else if (sign == "Turn right ahead") {
        moveBob(1);
        // Turn 90 degrees on the right
        rotateBob(-90);
    }
}

void publishRobotState(ros::Publisher& pub, State state)
{
    std_msgs::String state_msg;
    
    std::string state_string;

    switch (state) {
        case idle:
            state_string = "idle";
            break;
        case processing:
            state_string = "processing";
            break;
        case emergency_pressed:
            state_string = "emergency_pressed";
            break;
    }

    state_msg.data = state_string;
    pub.publish(state_msg);
}
/**
 * @brief Main function
 * 
 * @param argc 
 * @param argv 
 * @return int 
 * @details This function is the main function of the program. It starts the left and right motor PWM pins
 * and sets the speed of the motors to 0.0. It then starts the ROS node, and subscribes to the cmd_vel topic.
 * It then spins the node, which means that it waits for messages to be received on the cmd_vel topic. When a
 * message is received, the messageCallback function is called.
 */
int main(int argc, char** argv) {
        //Ros node Setup
        ros::init(argc, argv, "run_motor");

        //Run the gyroscope sensor
        wiringPiSetup();
        gy521.Initialize();
        ros::NodeHandle nh;

        ros::Publisher pub = nh.advertise<std_msgs::String>("main_node_status", 1000);
        //ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &joyStickCallback);
        //ros::Subscriber sub = nh.subscribe("cmd_vel", FREQUENCY, &messageCallback);
        //ros::Subscriber motor_speed_sub = nh.subscribe<std_msgs::Int32>("motor_speed", 10, motorSpeedCallback);
        ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu_data", 10, &imuDataCallback);
        ros::Subscriber qr_sub = nh.subscribe<std_msgs::Int32>("/usb_webcam/qr_data", 10 ,&qrCallback);
        ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
        ros::Subscriber autonomous_sub = nh.subscribe<std_msgs::Int32>("auto_data",10,&autoCallback);
        ros::Subscriber sign_sub = nh.subscribe<std_msgs::String>("Predicted_Sign", 10, &signCallback);
        state_bob = idle;  

        // Publish the robot's state
        publishRobotState(pub, state_bob);
        ros::spin();
        ROS_INFO_STREAM("spin");

        // CleanUp
        softPwmWrite(LEFT_PWM_PIN,0);
        digitalWrite(LEFT_PWM_PIN,LOW);
        softPwmWrite(RIGHT_PWM_PIN,0);
        digitalWrite(RIGHT_PWM_PIN,LOW);
        digitalWrite(RIGHT_DIRECTION_PIN,LOW);
        digitalWrite(LEFT_DIRECTION_PIN,LOW);

}
