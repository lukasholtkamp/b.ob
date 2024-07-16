// src/pid_node.cpp
#include "bob_pid/pid_node.hpp"

using std::placeholders::_1;

PID::PID()
    : Node("pid_node"), dt(0.1), last_time(this->now())
{
    // Load parameters from the YAML file
    this->declare_parameter<float>("Kp", 0.7);
    this->declare_parameter<float>("Ki", 0.0);
    this->declare_parameter<float>("Kd", 0.1);
    this->declare_parameter<float>("min_output", -0.4);
    this->declare_parameter<float>("max_output", 0.4);
    this->declare_parameter<float>("min_sum", -2.0);
    this->declare_parameter<float>("max_sum", 2.0);

    this->get_parameter("Kp", Kp);
    this->get_parameter("Ki", Ki);
    this->get_parameter("Kd", Kd);
    this->get_parameter("min_output", min_output);
    this->get_parameter("max_output", max_output);
    this->get_parameter("min_sum", min_sum);
    this->get_parameter("max_sum", max_sum);

    // Implementing the Subscriber for odom topic
    odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10, std::bind(&PID::odom_callback, this, _1));

    // Implementing the Subscriber for the Button and Axes request
    gamepad_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&PID::joy_callback, this, _1));

    // Implementing the Subscriber for the Button and Axes request
    setpoint_subscriber = this->create_subscription<std_msgs::msg::Float32>(
        "setpoint", 10, std::bind(&PID::setpoint_callback, this, _1));

    // Implementing the Publisher for the cmd_vel topic
    twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>("diffbot_base_controller/cmd_vel_unstamped", 10);

    // Implementing the Publisher for the initial_pose topic
    initial_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initial_pose", 10);

    pid_cmd_publisher = this->create_publisher<std_msgs::msg::Float32>("pid_cmd_vel", 10);

    // Initialize the PID controller
    initialize_pid(Kp, Ki, Kd, min_output, max_output, min_sum, max_sum);
}

void PID::initialize_pid(float Kp, float Ki, float Kd, float min_output, float max_output, float min_sum, float max_sum)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->error = 0;
    this->setpoint = 0;
    this->sum = 0;
    this->previous_error = 0;
    this->min_output = min_output;
    this->max_output = max_output;
    this->min_sum = min_sum;
    this->max_sum = max_sum;
    std::cout << "Kp: " << Kp << "\n"
              << "Ki: " << Ki << "\n"
              << "Kd: " << Kd << std::endl;
}

float PID::normalize_angle(float angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

void PID::setpoint_callback(const std_msgs::msg::Float32 &setpoint_msg)
{
    prev_setpoint = setpoint;
    setpoint = setpoint_msg.data;
}

void PID::joy_callback(const sensor_msgs::msg::Joy &joy_msg)
{
    // Intentionally left blank as pid_on is not used anymore
}

void PID::odom_callback(const nav_msgs::msg::Odometry &odom)
{
    if (abs(prev_setpoint - setpoint) > 0)
    {
        error = 0.0;
        sum = 0.0;
    }

    // Get the current time and calculate dt
    rclcpp::Time current_time = this->now();
    dt = (current_time - last_time).seconds();
    last_time = current_time;

    // Extract quaternion
    auto quaternion = odom.pose.pose.orientation;

    // Convert quaternion to tf2::Quaternion
    tf2::Quaternion tf2_quaternion(
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w);

    // Convert tf2::Quaternion to Euler angles
    tf2::Matrix3x3 mat(tf2_quaternion);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    // Round setpoint and yaw to 2 decimal places
    setpoint = round(setpoint * 100.0) / 100.0;
    yaw = round(yaw * 100.0) / 100.0;

    std::cout << "setpoint : " << setpoint << std::endl;
    std::cout << "yaw : " << yaw << std::endl;

    // Normalize yaw and setpoint
    yaw = normalize_angle(yaw);
    setpoint = normalize_angle(setpoint);

    error = setpoint - yaw;
    error = normalize_angle(error); // Ensure the shortest path is taken

    // Round error to 1 decimal place
    error = round(error * 10.0) / 10.0;

    std::cout << "error : " << error << std::endl;

    // Apply deadband to error
    if (std::abs(error) < tolerance)
    {
        error = 0.0;
    }

    sum += error * dt;

    // Round sum to 1 decimal place
    sum = round(sum * 10.0) / 10.0;

    std::cout << "sum : " << sum << std::endl;

    // Implement anti-windup
    if (sum > max_sum)
    {
        sum = max_sum;
    }
    else if (sum < min_sum)
    {
        sum = min_sum;
    }

    float derivative = (error - previous_error) / dt;
    float output = Kp * error + Ki * sum + Kd * derivative;

    // Round output to 1 decimal place
    output = round(output * 10.0) / 10.0;

    std::cout << "output : " << output << "\n\n"
              << std::endl;

    // Clamp the output
    if (output > max_output)
    {
        output = max_output;
    }
    else if (output < min_output)
    {
        output = min_output;
    }

    previous_error = error;

    // Publish twist message based on PID output
    auto pid_cmd_msg = std_msgs::msg::Float32();
    pid_cmd_msg.data = output;
    pid_cmd_publisher->publish(pid_cmd_msg);
}

void PID::publish_initial_pose(double x, double y, const tf2::Quaternion &quaternion)
{
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = this->now(); // Set the timestamp to the current time
    pose_msg.header.frame_id = "odom";
    pose_msg.pose.pose.position.x = x;
    pose_msg.pose.pose.position.y = y;
    pose_msg.pose.pose.orientation.x = quaternion.x();
    pose_msg.pose.pose.orientation.y = quaternion.y();
    pose_msg.pose.pose.orientation.z = quaternion.z();
    pose_msg.pose.pose.orientation.w = quaternion.w();
    initial_pose_publisher->publish(pose_msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PID>());
    rclcpp::shutdown();
    return 0;
}
