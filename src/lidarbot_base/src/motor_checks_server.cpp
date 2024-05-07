// Server node runs motor tests to confirm that the motor is working correctly by moving both motors forward

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "lidarbot_base/motor_encoder.h"

extern int left_wheel_pulse_count;
extern int right_wheel_pulse_count;
extern int left_wheel_direction;
extern int right_wheel_direction;
extern int* pi_int;

// Reset pulse counters
void reset_pulse_counters()
{
    right_wheel_pulse_count = 0;
    left_wheel_pulse_count = 0;
}

// Move a specified motor forward
bool move_motor(int motor_pin)
{   
    // Motor_id
    // MOTORA - 0 (left motor)
    // MOTORB - 1 (right motor)
    reset_pulse_counters();
    sleep(2);
    
    // Set motor directions
    if(motor_pin == LEFT_PWM_PIN) 
        gpio_write(*pi_int,LEFT_DIRECTION_PIN, CCW);
    else
        gpio_write(*pi_int,LEFT_DIRECTION_PIN, CW);

    // Move motor FORWARD for 2 seconds at 50% speed
    set_PWM_dutycycle(*pi_int,LEFT_PWM_PIN,50);
    sleep(2);
    set_PWM_dutycycle(*pi_int,LEFT_PWM_PIN,0);

    // Motor counts should be greater than 0 to confirm that the motor moved forward
    if (motor_pin == LEFT_PWM_PIN) {
        if (left_wheel_pulse_count> 0) {
            return true;
        } else return false;
    }
    if (motor_pin == RIGHT_PWM_PIN) {
        if (right_wheel_pulse_count > 0) {
            return true;
        } return false;
    }
}

// DDS helps pass the request and response between client and server
void checkMotors(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    // Prepare response
    response->success = true;
    response->message = "";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received request to check motors...");

    // Left motor check
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Checking left motor...");
    auto left_motor_passed = move_motor(LEFT_PWM_PIN);
    if (!left_motor_passed) {
        response->success = false;
        response->message += "Left motor check failed, confirm motor wiring.";
    }

    // Right motor check
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Checking right motor...");
    auto right_motor_passed = move_motor(RIGHT_PWM_PIN);
    if (!right_motor_passed) {
        response->success = false;
        response->message += "Right motor check failed, confirm motor wiring.";
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response...");
}

int main(int argc, char **argv)
{

    int pi = pigpio_start(NULL,NULL);
    pi_int = &pi;
    
    // Setup gpio outputs
    set_mode(pi,LEFT_DIRECTION_PIN,PI_OUTPUT);
    set_mode(pi,RIGHT_DIRECTION_PIN,PI_OUTPUT);
    set_mode(pi,LEFT_PWM_PIN,PI_OUTPUT);
    set_mode(pi,RIGHT_PWM_PIN,PI_OUTPUT);

    // Setup GPIO encoder interrupt and direction pins
    set_mode(pi,LEFT_ALARM_PIN,PI_INPUT);
    set_mode(pi,RIGHT_ALARM_PIN,PI_INPUT);
    set_mode(pi,LEFT_ENCODER_PIN,PI_INPUT);
    set_mode(pi,RIGHT_ENCODER_PIN,PI_INPUT);

    // Setup pull up resistors on encoder interrupt pins
    set_pull_up_down(pi,LEFT_ALARM_PIN,PI_PUD_DOWN);
    set_pull_up_down(pi,RIGHT_ALARM_PIN,PI_PUD_DOWN);
    set_pull_up_down(pi,LEFT_ENCODER_PIN,PI_PUD_UP);
    set_pull_up_down(pi,RIGHT_ENCODER_PIN,PI_PUD_UP);

    // Initialize encoder interrupts for falling signal states
    callback(pi,LEFT_ENCODER_PIN,RISING_EDGE,left_wheel_pulse);
    callback(pi,RIGHT_ENCODER_PIN,RISING_EDGE,right_wheel_pulse);

    gpio_write(pi,LEFT_DIRECTION_PIN,CCW);
    gpio_write(pi,RIGHT_DIRECTION_PIN,CW);

    // Initialize the rclcpp library
    rclcpp::init(argc, argv);

    // Create a shared pointer to a Node type and name it "motor_checks_server"
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("motor_checks_server");

    // Create a "checks" service with a checkMotors callback
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service = 
        node->create_service<std_srvs::srv::Trigger>("checks", &checkMotors);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to check motors");

    // Spin the node until it's terminated
    rclcpp::spin(node);
    rclcpp::shutdown();
}