// Based on: https://github.com/TheNoobInventor/lidarbot/blob/main/lidarbot_base/src/motor_encoder.c
// Date of Retrieval: 17.05.2024

#include "motor.hpp"
#include <math.h>
#include <iostream>

// Initialize pulse counters
int left_wheel_pulse_count = 0;
int right_wheel_pulse_count = 0;
u_int32_t _high_tick = 0;
u_int32_t _period = 0;
double radius = 0.08255;
double ticks_per_rev = 98.0;

// ID for pi obtained from running pigio package
extern int pi_sig;

// Read wheel encoder values
void read_encoder_values(int *left_encoder_value, int *right_encoder_value)
{
    *left_encoder_value = left_wheel_pulse_count;
    *right_encoder_value = right_wheel_pulse_count;
}

/** 
 * @brief calculate difference between two times
 * @param o_tick old tick
 * @param c_tick current tick
 * @return time difference in microseconds
 */
u_int32_t tick_diff(u_int32_t o_tick, u_int32_t c_tick){
  return c_tick-o_tick;
}

// Left wheel callback function
void left_wheel_pulse(int pi, u_int user_gpio, u_int level, uint32_t tick)
{
    (void)user_gpio;
    // Left wheel direction
    // CCW - forward
    // CW - backward

    // Read encoder direction value for left wheel
    int left_wheel_direction = gpio_read(pi, LEFT_DIRECTION_PIN);

    if (left_wheel_direction == CCW)
    {
        left_wheel_pulse_count++;
    }
    else
    {
        left_wheel_pulse_count--;
    }

    // rising edge
    if(level == 1){

        if(_high_tick != 0){
            // find period between last pulse and this pulse
            _period = tick_diff(_high_tick,tick);

            double freq = 1000000.0/double(_period);
            double speed = radius*2*M_PI*(freq/ticks_per_rev);
            std::cout << "Left Wheel Speed Encoder: " << speed << std::endl;
        }
        _high_tick = tick;
    }

}

// Right wheel callback function
void right_wheel_pulse(int pi, u_int user_gpio, u_int level, uint32_t tick)
{
    (void)user_gpio;
    (void)level;
    (void)tick;
    // Right wheel direction
    // CW - forward,
    // CCW - backward

    // Read encoder direction value for right wheel
    int right_wheel_direction = gpio_read(pi, RIGHT_DIRECTION_PIN);

    if (right_wheel_direction == CW)
    {
        right_wheel_pulse_count++;
    }
    else
    {
        right_wheel_pulse_count--;
    }
}

// Set each motor speed from the respective velocity command interface
void set_motor_speeds(int pi, double left_wheel_command, double right_wheel_command)
{

    // Set motor directions
    if (left_wheel_command > 0)
    {
        gpio_write(pi, LEFT_DIRECTION_PIN, CCW);
    }

    else
    {
        gpio_write(pi, LEFT_DIRECTION_PIN, CW);
    }

    if (right_wheel_command > 0)
    {
        gpio_write(pi, RIGHT_DIRECTION_PIN, CW);
    }

    else
    {
        gpio_write(pi, RIGHT_DIRECTION_PIN, CCW);
    }

    // Send PWM signals to motors
    set_PWM_dutycycle(pi, LEFT_PWM_PIN, (int)abs(left_wheel_command));
    set_PWM_dutycycle(pi, RIGHT_PWM_PIN, (int)abs(right_wheel_command));
}


void handler(int signo)
{
    (void)signo;
    set_PWM_dutycycle(pi_sig, LEFT_PWM_PIN, 0);
    set_PWM_dutycycle(pi_sig, RIGHT_PWM_PIN, 0);

    exit(0);
}