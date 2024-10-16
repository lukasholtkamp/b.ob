// Based on: https://github.com/ros2/teleop_twist_joy/blob/rolling/src/teleop_twist_joy.cpp
// Date of Retrieval: 17.05.2024

#include <cinttypes>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <thread>
#include <string>
#include <iostream>

#include "rcutils/logging_macros.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "std_msgs/msg/float32.hpp"

#include "bob_teleop/teleop_twist_joy.hpp"

#define ROS_INFO_NAMED RCUTILS_LOG_INFO_NAMED
#define ROS_INFO_COND_NAMED RCUTILS_LOG_INFO_EXPRESSION_NAMED

namespace bob_teleop
{

  /**
   * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
   * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
   * directly into base nodes.
   * Implementing and declare all neccesary parameters, functions, variables, subscribers and publishers.
   */
  struct TeleopTwistJoy::Impl
  {
    //! Callback function for the joy_sub subscriber to listen to the joy topic
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy);

    //! Callback function for the jointstate subscriber to listen to the joy states
    void jointstate_callback(const sensor_msgs::msg::JointState &state);

    //! Callback function for the odom_subscriber to read from the /odometry/filtered topic
    void odom_callback(const nav_msgs::msg::Odometry &odom);

    //! Callback function for the pid_cmd_subscriber to read from the pid_cmd_vel topic
    void pid_callback(const std_msgs::msg::Float32 &pid_msg);

    //! Function to publish calculated velocity to cmd_vel_pub
    void send_cmd_vel_msg(const sensor_msgs::msg::Joy::SharedPtr, const std::string &which_map);

    //! Function for filling out the cmd_vel_msg
    void fill_cmd_vel_msg(
        const sensor_msgs::msg::Joy::SharedPtr, const std::string &which_map,
        geometry_msgs::msg::Twist *cmd_vel_msg);

    //! Subscriber to read from the odom topic
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;

    //! Subscriber to read from the pid topic
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pid_subscriber;

    //! Subscriber to read wheel movements and velocities
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointstate_subscriber;

    //! Publisher for publishing the new setpoint on the setpoint topic
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr setpoint_publisher;

    //! Subscriber to listen to joy topic for the speed controlling axes
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

    //! Publisher for sending the command velocity on the /cmd_vel topic
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

    //! Publisher for sending the time stamped command velocity on the /cmd_vel topic
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_pub;

    //! Clock used for time stamping
    rclcpp::Clock::SharedPtr clock;

    //! Boolean to decide whether PID controller is used or not
    bool use_pid;

    //! Boolean to decide whether change the setpoint for the PID or not
    bool read_value;

    //! Boolean to decide whether the command velocity is time stmapped or not
    bool publish_stamped_twist;

    //! ID for the frame
    std::string frame_id;

    //! Boolean for whether a enable button needs to be held for the messages to be sent
    bool require_enable_button;

    //! Integer of the button used for the enable functionality
    int64_t enable_button;

    //! Integer of the button used for a turbo functionality
    int64_t enable_turbo_button;

    //! Boolean to inverse the axis directions
    bool inverted_reverse;

    //! Axis mapping from joystick for linear motion
    std::map<std::string, int64_t> axis_linear_map;

    // Added linear reverse map
    //! Axis mapping from joystick for reverse driving
    std::map<std::string, int64_t> axis_linear_reverse_map;

    //! Scale map to scale the linear limits
    std::map<std::string, std::map<std::string, double>> scale_linear_map;

    // Added scale linear reverse map
    //! Scale map to scale the reverse linear limits
    std::map<std::string, std::map<std::string, double>> scale_linear_reverse_map;

    //! Axis mapping from joystick for angular motion
    std::map<std::string, int64_t> axis_angular_map;

    //! Scale map to scale the angular limits
    std::map<std::string, std::map<std::string, double>> scale_angular_map;

    //! Boolean used for sending 0 command velocity for the enable driving button functionality
    bool sent_disable_msg;

    //! Boolean used for checking if B.ob is reversing
    bool reversing_check;

    //! Current Yaw angle from Bob
    double yaw;
    //! Previous Yaw angle from Bob
    double yaw_prev;
    //! PID cmd vel from the PID node
    float pid_cmd_vel = 0.0;
    //! Right wheel velocity
    float right_vel = 0.0;
    //! Left wheel velocity
    float left_vel = 0.0;
  };

  /**
   * Constructs TeleopTwistJoy.
   */
  TeleopTwistJoy::TeleopTwistJoy(const rclcpp::NodeOptions &options)
      : rclcpp::Node("teleop_twist_joy_node", options)
  {
    pimpl_ = new Impl;

    pimpl_->clock = this->get_clock();

    pimpl_->use_pid = this->declare_parameter("use_pid", false);

    pimpl_->publish_stamped_twist = this->declare_parameter("publish_stamped_twist", false);
    pimpl_->frame_id = this->declare_parameter("frame", "teleop_twist_joy");

    if (pimpl_->publish_stamped_twist)
    {
      pimpl_->cmd_vel_stamped_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(
          "cmd_vel", 10);
    }
    else
    {
      pimpl_->cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

    pimpl_->setpoint_publisher = this->create_publisher<std_msgs::msg::Float32>("setpoint", 10);

    pimpl_->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", rclcpp::QoS(10),
        std::bind(&TeleopTwistJoy::Impl::joy_callback, this->pimpl_, std::placeholders::_1));

    // subscriber to read wheel movements
    pimpl_->jointstate_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&TeleopTwistJoy::Impl::jointstate_callback, this->pimpl_, std::placeholders::_1));

    pimpl_->odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10, std::bind(&TeleopTwistJoy::Impl::odom_callback, this->pimpl_, std::placeholders::_1));

    pimpl_->pid_subscriber = this->create_subscription<std_msgs::msg::Float32>(
        "pid_cmd_vel", 10, std::bind(&TeleopTwistJoy::Impl::pid_callback, this->pimpl_, std::placeholders::_1));

    pimpl_->require_enable_button = this->declare_parameter("require_enable_button", true);

    pimpl_->enable_button = this->declare_parameter("enable_button", 5);

    pimpl_->enable_turbo_button = this->declare_parameter("enable_turbo_button", -1);

    pimpl_->inverted_reverse = this->declare_parameter("inverted_reverse", false);

    std::map<std::string, int64_t> default_linear_map{
        {"x", 5L},
        {"y", -1L},
        {"z", -1L},
    };
    this->declare_parameters("axis_linear", default_linear_map);
    this->get_parameters("axis_linear", pimpl_->axis_linear_map);

    // Default values for reverse map
    std::map<std::string, int64_t> default_linear_reverse_map{
        {"x", -1L},
        {"y", -1L},
        {"z", -1L},
    };
    this->declare_parameters("axis_linear_reverse", default_linear_reverse_map);
    this->get_parameters("axis_linear_reverse", pimpl_->axis_linear_reverse_map);

    std::map<std::string, int64_t> default_angular_map{
        {"yaw", 2L},
        {"pitch", -1L},
        {"roll", -1L},
    };
    this->declare_parameters("axis_angular", default_angular_map);
    this->get_parameters("axis_angular", pimpl_->axis_angular_map);

    std::map<std::string, double> default_scale_linear_normal_map{
        {"x", 0.5},
        {"y", 0.0},
        {"z", 0.0},
    };
    this->declare_parameters("scale_linear", default_scale_linear_normal_map);
    this->get_parameters("scale_linear", pimpl_->scale_linear_map["normal"]);

    // Default reverse scale values
    std::map<std::string, double> default_scale_linear_reverse_normal_map{
        {"x", 0.5},
        {"y", 0.0},
        {"z", 0.0},
    };
    this->declare_parameters("scale_linear_reverse", default_scale_linear_reverse_normal_map);
    this->get_parameters("scale_linear_reverse", pimpl_->scale_linear_reverse_map["normal"]);

    std::map<std::string, double> default_scale_linear_turbo_map{
        {"x", 1.0},
        {"y", 0.0},
        {"z", 0.0},
    };
    this->declare_parameters("scale_linear_turbo", default_scale_linear_turbo_map);
    this->get_parameters("scale_linear_turbo", pimpl_->scale_linear_map["turbo"]);

    // Default reverse turbo map
    std::map<std::string, double> default_scale_linear_reverse_turbo_map{
        {"x", 1.0},
        {"y", 0.0},
        {"z", 0.0},
    };
    this->declare_parameters("scale_linear_reverse_turbo", default_scale_linear_reverse_turbo_map);
    this->get_parameters("scale_linear_reverse_turbo", pimpl_->scale_linear_reverse_map["turbo"]);

    std::map<std::string, double> default_scale_angular_normal_map{
        {"yaw", 0.5},
        {"pitch", 0.0},
        {"roll", 0.0},
    };
    this->declare_parameters("scale_angular", default_scale_angular_normal_map);
    this->get_parameters("scale_angular", pimpl_->scale_angular_map["normal"]);

    std::map<std::string, double> default_scale_angular_turbo_map{
        {"yaw", 1.0},
        {"pitch", 0.0},
        {"roll", 0.0},
    };
    this->declare_parameters("scale_angular_turbo", default_scale_angular_turbo_map);
    this->get_parameters("scale_angular_turbo", pimpl_->scale_angular_map["turbo"]);

    ROS_INFO_COND_NAMED(
        pimpl_->require_enable_button, "TeleopTwistJoy",
        "Teleop enable button %" PRId64 ".", pimpl_->enable_button);

    ROS_INFO_COND_NAMED(
        pimpl_->use_pid, "TeleopTwistJoy",
        "PID enabled.");

    ROS_INFO_COND_NAMED(
        pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
        "Turbo on button %" PRId64 ".", pimpl_->enable_turbo_button);
    ROS_INFO_COND_NAMED(
        pimpl_->inverted_reverse, "TeleopTwistJoy", "%s", "Teleop enable inverted reverse.");

    for (std::map<std::string, int64_t>::iterator it = pimpl_->axis_linear_map.begin();
         it != pimpl_->axis_linear_map.end(); ++it)
    {
      ROS_INFO_COND_NAMED(
          it->second != -1L, "TeleopTwistJoy", "Linear axis %s on %" PRId64 " at scale %f.",
          it->first.c_str(), it->second, pimpl_->scale_linear_map["normal"][it->first]);
      ROS_INFO_COND_NAMED(
          pimpl_->enable_turbo_button >= 0 && it->second != -1, "TeleopTwistJoy",
          "Turbo for linear axis %s is scale %f.", it->first.c_str(),
          pimpl_->scale_linear_map["turbo"][it->first]);
    }

    // For loop to print the reverse axis details
    for (std::map<std::string, int64_t>::iterator it = pimpl_->axis_linear_reverse_map.begin();
         it != pimpl_->axis_linear_reverse_map.end(); ++it)
    {
      ROS_INFO_COND_NAMED(
          it->second != -1L, "TeleopTwistJoy", "Reverse Linear axis %s on %" PRId64 " at scale %f.",
          it->first.c_str(), it->second, pimpl_->scale_linear_reverse_map["normal"][it->first]);
      ROS_INFO_COND_NAMED(
          pimpl_->enable_turbo_button >= 0 && it->second != -1, "TeleopTwistJoy",
          "Turbo for reverse linear axis %s is scale %f.", it->first.c_str(),
          pimpl_->scale_linear_reverse_map["turbo"][it->first]);
    }

    for (std::map<std::string, int64_t>::iterator it = pimpl_->axis_angular_map.begin();
         it != pimpl_->axis_angular_map.end(); ++it)
    {
      ROS_INFO_COND_NAMED(
          it->second != -1L, "TeleopTwistJoy", "Angular axis %s on %" PRId64 " at scale %f.",
          it->first.c_str(), it->second, pimpl_->scale_angular_map["normal"][it->first]);
      ROS_INFO_COND_NAMED(
          pimpl_->enable_turbo_button >= 0 && it->second != -1, "TeleopTwistJoy",
          "Turbo for angular axis %s is scale %f.", it->first.c_str(),
          pimpl_->scale_angular_map["turbo"][it->first]);
    }

    pimpl_->sent_disable_msg = false;

    // set reverse boolean to false
    pimpl_->reversing_check = false;

    auto param_callback =
        [this](std::vector<rclcpp::Parameter> parameters)
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      // Loop to assign changed parameters to the member variables
      for (const auto &parameter : parameters)
      {
        if (parameter.get_name() == "require_enable_button")
        {
          this->pimpl_->require_enable_button = parameter.get_value<rclcpp::PARAMETER_BOOL>();
        }
        else if (parameter.get_name() == "inverted_reverse")
        {
          this->pimpl_->inverted_reverse = parameter.get_value<rclcpp::PARAMETER_BOOL>();
        }
        else if (parameter.get_name() == "use_pid")
        {
          this->pimpl_->use_pid = parameter.get_value<rclcpp::PARAMETER_BOOL>();
        }
        else if (parameter.get_name() == "enable_button")
        {
          this->pimpl_->enable_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "enable_turbo_button")
        {
          this->pimpl_->enable_turbo_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "axis_linear.x")
        {
          this->pimpl_->axis_linear_map["x"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "axis_linear.y")
        {
          this->pimpl_->axis_linear_map["y"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "axis_linear.z")
        {
          this->pimpl_->axis_linear_map["z"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }

        // Read yaml file values
        else if (parameter.get_name() == "axis_linear_reverse.x")
        {
          this->pimpl_->axis_linear_reverse_map["x"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "axis_linear_reverse.y")
        {
          this->pimpl_->axis_linear_reverse_map["y"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "axis_linear_reverse.z")
        {
          this->pimpl_->axis_linear_reverse_map["z"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }

        else if (parameter.get_name() == "axis_angular.yaw")
        {
          this->pimpl_->axis_angular_map["yaw"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "axis_angular.pitch")
        {
          this->pimpl_->axis_angular_map["pitch"] =
              parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "axis_angular.roll")
        {
          this->pimpl_->axis_angular_map["roll"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "scale_linear_turbo.x")
        {
          this->pimpl_->scale_linear_map["turbo"]["x"] =
              parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_linear_turbo.y")
        {
          this->pimpl_->scale_linear_map["turbo"]["y"] =
              parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_linear_turbo.z")
        {
          this->pimpl_->scale_linear_map["turbo"]["z"] =
              parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_linear.x")
        {
          this->pimpl_->scale_linear_map["normal"]["x"] =
              parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_linear.y")
        {
          this->pimpl_->scale_linear_map["normal"]["y"] =
              parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_linear.z")
        {
          this->pimpl_->scale_linear_map["normal"]["z"] =
              parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }

        // Read yaml file values
        else if (parameter.get_name() == "scale_linear_reverse.x")
        {
          this->pimpl_->scale_linear_reverse_map["normal"]["x"] =
              parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_linear_reverse.y")
        {
          this->pimpl_->scale_linear_reverse_map["normal"]["y"] =
              parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_linear_reverse.z")
        {
          this->pimpl_->scale_linear_reverse_map["normal"]["z"] =
              parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }

        else if (parameter.get_name() == "scale_angular_turbo.yaw")
        {
          this->pimpl_->scale_angular_map["turbo"]["yaw"] =
              parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular_turbo.pitch")
        {
          this->pimpl_->scale_angular_map["turbo"]["pitch"] =
              parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular_turbo.roll")
        {
          this->pimpl_->scale_angular_map["turbo"]["roll"] =
              parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular.yaw")
        {
          this->pimpl_->scale_angular_map["normal"]["yaw"] =
              parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular.pitch")
        {
          this->pimpl_->scale_angular_map["normal"]["pitch"] =
              parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular.roll")
        {
          this->pimpl_->scale_angular_map["normal"]["roll"] =
              parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
      }
      return result;
    };

    callback_handle = this->add_on_set_parameters_callback(param_callback);
  }

  TeleopTwistJoy::~TeleopTwistJoy()
  {
    delete pimpl_;
  }

  double get_val(
      const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::map<std::string, int64_t> &axis_map,
      const std::map<std::string, double> &scale_map, const std::string &fieldname)
  {
    if (axis_map.find(fieldname) == axis_map.end() ||
        axis_map.at(fieldname) == -1L ||
        scale_map.find(fieldname) == scale_map.end() ||
        static_cast<int>(joy_msg->axes.size()) <= axis_map.at(fieldname))
    {
      return 0.0;
    }

    if (axis_map.at(fieldname) == 4 || axis_map.at(fieldname) == 5)
    {
      return (std::round(0.5 * (1 - joy_msg->axes[axis_map.at(fieldname)]) * 1000.0) / 1000.0) * scale_map.at(fieldname);
    }

    return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
  }

  void TeleopTwistJoy::Impl::send_cmd_vel_msg(
      const sensor_msgs::msg::Joy::SharedPtr joy_msg,
      const std::string &which_map)
  {
    if (publish_stamped_twist)
    {
      auto cmd_vel_stamped_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
      cmd_vel_stamped_msg->header.stamp = clock->now();
      cmd_vel_stamped_msg->header.frame_id = frame_id;
      fill_cmd_vel_msg(joy_msg, which_map, &cmd_vel_stamped_msg->twist);
      cmd_vel_stamped_pub->publish(std::move(cmd_vel_stamped_msg));
    }
    else
    {
      auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
      fill_cmd_vel_msg(joy_msg, which_map, cmd_vel_msg.get());
      cmd_vel_pub->publish(std::move(cmd_vel_msg));
    }
    sent_disable_msg = false;
  }

  void TeleopTwistJoy::Impl::fill_cmd_vel_msg(
      const sensor_msgs::msg::Joy::SharedPtr joy_msg,
      const std::string &which_map,
      geometry_msgs::msg::Twist *cmd_vel_msg)
  {
    // Instantiate variables
    double reverse;
    double forward;

    double ang_z = 0;
    double lin_x = 0;

    // Check if using triggers for driving forward or backward
    if (axis_linear_reverse_map.at("x") == 4 || axis_linear_reverse_map.at("x") == 5 || axis_linear_map.at("x") == 4 || axis_linear_map.at("x") == 5)
    {
      // Map the trigger values to 0 to 1
      reverse = std::round(-0.5 * (1 - joy_msg->axes[axis_linear_reverse_map.at("x")]) * 1000000.0) / 1000000.0;
      forward = std::round(0.5 * (1 - joy_msg->axes[axis_linear_map.at("x")]) * 1000000.0) / 1000000.0;

      // Ensure that only driving forward or backward
      if (reverse < 0 && forward == 0)
      {
        lin_x = reverse * scale_linear_reverse_map[which_map].at("x");
      }

      else if (reverse == 0 && forward > 0)
      {
        lin_x = get_val(joy_msg, axis_linear_map, scale_linear_map[which_map], "x");
      }
    }

    // Default method in original code
    else
    {
      lin_x = get_val(joy_msg, axis_linear_map, scale_linear_map[which_map], "x");
    }

    // If driving command is forward but B.ob is still reversing, do not move (to prevent wheelie)
    if (lin_x > 0 && reversing_check)
    {
      lin_x = 0;
    }

    // If using the PID controller
    if (use_pid)
    {
      // Check that the joy stick is not being moved with a certain tolerance
      if (abs(joy_msg->axes[axis_angular_map.at("yaw")]) > 0.0099)
      {
        ang_z = get_val(joy_msg, axis_angular_map, scale_angular_map[which_map], "yaw");
        read_value = true;
      }

      else
      {
        // Once the joy stick has stopped moving, get the new setpoint
        if (read_value)
        {
          // Set the setpoint only when the wheels do not move
          if (left_vel == 0 && right_vel == 0)
          {
            auto setpoint_msg = std_msgs::msg::Float32();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            setpoint_msg.data = yaw;
            setpoint_publisher->publish(setpoint_msg);
            read_value = false;
          }
          // If the wheels are still moving then send no turn velocity or else it will try go to the old setpoint
          else
          {
            ang_z = 0.0;
          }
        }
        // If the setpoint has been set, maintain this direction
        else
        {
          ang_z = pid_cmd_vel;
        }
      }
    }
    else
    {
      ang_z = get_val(joy_msg, axis_angular_map, scale_angular_map[which_map], "yaw");
    }

    cmd_vel_msg->linear.x = lin_x;
    cmd_vel_msg->linear.y = get_val(joy_msg, axis_linear_map, scale_linear_map[which_map], "y");
    cmd_vel_msg->linear.z = get_val(joy_msg, axis_linear_map, scale_linear_map[which_map], "z");
    cmd_vel_msg->angular.z = (lin_x < 0.0 && inverted_reverse) ? -ang_z : ang_z;
    cmd_vel_msg->angular.y = get_val(joy_msg, axis_angular_map, scale_angular_map[which_map], "pitch");
    cmd_vel_msg->angular.x = get_val(joy_msg, axis_angular_map, scale_angular_map[which_map], "roll");
  }

  void TeleopTwistJoy::Impl::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    if (enable_turbo_button >= 0 &&
        static_cast<int>(joy_msg->buttons.size()) > enable_turbo_button &&
        joy_msg->buttons[enable_turbo_button])
    {
      send_cmd_vel_msg(joy_msg, "turbo");
    }
    else if (!require_enable_button || // NOLINT
             (static_cast<int>(joy_msg->buttons.size()) > enable_button &&
              joy_msg->buttons[enable_button]))
    {
      send_cmd_vel_msg(joy_msg, "normal");
    }
    else
    {
      // When enable button is released, immediately send a single no-motion command
      // in order to stop the robot.
      if (!sent_disable_msg)
      {
        // Initializes with zeros by default.
        if (publish_stamped_twist)
        {
          auto cmd_vel_stamped_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
          cmd_vel_stamped_msg->header.stamp = clock->now();
          cmd_vel_stamped_msg->header.frame_id = frame_id;
          cmd_vel_stamped_pub->publish(std::move(cmd_vel_stamped_msg));
        }
        else
        {
          auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
          cmd_vel_pub->publish(std::move(cmd_vel_msg));
        }
        sent_disable_msg = true;
      }
    }
  }

  void TeleopTwistJoy::Impl::jointstate_callback(const sensor_msgs::msg::JointState &state)
  {
    // Obtain the velocities from the wheels
    left_vel = state.velocity[0];
    right_vel = state.velocity[1];

    // Checks if both wheels are still reversing
    if (left_vel < 0.0 && right_vel < 0.0)
    {
      reversing_check = true;
    }
    else
    {
      reversing_check = false;
    }
  }

  void TeleopTwistJoy::Impl::odom_callback(const nav_msgs::msg::Odometry &odom)
  {
    yaw_prev = yaw;
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
    double roll, pitch;
    mat.getRPY(roll, pitch, yaw);
  }

  void TeleopTwistJoy::Impl::pid_callback(const std_msgs::msg::Float32 &pid_msg)
  {
    // Obtain the PID cmd vel from the published value
    pid_cmd_vel = pid_msg.data;
  }

} // namespace teleop_twist_joy

RCLCPP_COMPONENTS_REGISTER_NODE(bob_teleop::TeleopTwistJoy)