#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class XBoxController
{
public:
  XBoxController();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh;

  int linear, angular;
  double l_scale, a_scale;
  ros::Publisher vel_pub;
  ros::Subscriber joy_sub;
};


XBoxController::XBoxController():
  linear(1),
  angular(2)
{

  nh.param("axis_linear", linear, linear);
  nh.param("axis_angular", angular, angular);
  nh.param("scale_angular", a_scale, a_scale);
  nh.param("scale_linear", l_scale, l_scale);

  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &XBoxController::joyCallback, this);
}

void XBoxController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale*joy->axes[angular];
  twist.linear.x = l_scale*joy->axes[linear];  
  vel_pub.publish(twist);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xbox_controller");
  XBoxController xbox_controller;

  ros::spin();
}

