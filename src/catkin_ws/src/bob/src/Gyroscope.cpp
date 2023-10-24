#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
 // You need to install the necessary MPU6050 ROS package.
#include "MotorDriver.h"
#include "RunMotor.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mpu6050_interface");
    ros::NodeHandle nh;

    // Create a publisher to publish IMU data
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);

    // Create an MPU6050 object
    MD::GY521 gy521(0x68); 

    // Variables to hold sensor data
    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = "imu_link";

    ros::Rate loop_rate(50); // You may adjust the loop rate based on your sensor's capabilities

    while (ros::ok())
    {
        // Read data from the MPU6050 sensor
     //   mpu.readSensor();

        // Populate the IMU message
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.angular_velocity.x = gy521.ReadRawData(GYRO_XOUT_H);
        imu_msg.angular_velocity.y = gy521.ReadRawData(GYRO_YOUT_H);
        imu_msg.angular_velocity.z = gy521.ReadRawData(GYRO_ZOUT_H);
        imu_msg.linear_acceleration.x = gy521.ReadRawData(ACCEL_XOUT_H);
        imu_msg.linear_acceleration.y = gy521.ReadRawData(ACCEL_YOUT_H);
        imu_msg.linear_acceleration.z = gy521.ReadRawData(ACCEL_ZOUT_H);


        // Publish the IMU data
        imu_pub.publish(imu_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
