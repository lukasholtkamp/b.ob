#ifndef X_BOX_CONTROLLER_H
#define X_BOX_CONTROLLER_H

  #include <ros/ros.h> //<-- Used for the ROS environment
  #include <geometry_msgs/Twist.h> //<-- Used to receive messages from the cmd_vel topic
  #include <sensor_msgs/Joy.h> //<-- Used to receive messages from the joy topic
  #include <ostream> //<-- Used to print to the console

  //--Button configuration according to XBoxOne Controller
  #define A_BUTTON 0
  #define B_BUTTON 1
  #define Y_BUTTON 3
  #define X_BUTTON 2
  #define RIGHT_SHOULDER_BUTTON 5
  #define LEFT_SHOULDER_BUTTON 4

  //--Axes configuration according to XBoxOne Controller
  #define ANGULAR_VEL 0
  #define JOY_AXIS_RT 5
  #define JOY_AXIS_LT 2

  namespace XB{
    /**
     * @brief Motor class
     * @details This class is used to control the motors of the robot. The class is used to set the speed and direction of the motors.
     */
    class XBoxController {
    private:

      void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
      ros::NodeHandle nh; //<-- Node handle

      int linear, angular; //<-- Linear and angular values
      double lScale, aScale; //<-- Scale values
      ros::Publisher velPub; //<-- Publisher
      ros::Subscriber joy_sub; //<-- Subscriber

    public:

      XBoxController();
      void setGearChanger(int &gearChanger, int shoulderButton);
      int getGearChanger();
      double setLinearSpeed(geometry_msgs::Twist twist, const sensor_msgs::Joy::ConstPtr& joy);
    };
  }

#endif // MOTOR_DRIVER_H

