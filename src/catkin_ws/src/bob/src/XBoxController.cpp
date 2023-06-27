#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <ostream>

#define B_BUTTON 1

#define ANGULAR_VEL 0
#define JOY_AXIS_RT 4
#define JOY_AXIS_LT 5

class XBoxController
{
public:
  XBoxController();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh;

  int linear, angular;
  int b_button;
  double l_scale, a_scale;
  ros::Publisher vel_pub;
  ros::Subscriber joy_sub;
};


XBoxController::XBoxController():
  linear(JOY_AXIS_RT),
  angular(ANGULAR_VEL),
  b_button(B_BUTTON)
{

  nh.param("axis_linear", linear, linear);
  nh.param("axis_angular", angular, angular);
  nh.param("scale_angular", a_scale, a_scale);
  nh.param("scale_linear", l_scale, l_scale);
  nh.param("buttons", b_button, b_button);

  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &XBoxController::joyCallback, this);
}

void XBoxController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale*joy->axes[ANGULAR_VEL];
  twist.linear.x = l_scale*joy->axes[JOY_AXIS_RT];
  twist.linear.x = (1+ (twist.linear.x*-1))/2;
  auto button = joy->buttons[B_BUTTON];
  if(button == 1){
  	twist.linear.x = 0;
  	twist.angular.z = 0;
  }
  vel_pub.publish(twist);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xbox_controller");
  XBoxController xbox_controller;

  ros::spin();
}

