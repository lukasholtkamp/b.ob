#include "lidarbot_base/motor_encoder.h"
#include <math.h>

// Initialize pulse counters
int left_wheel_pulse_count = 0;
int right_wheel_pulse_count = 0;

// Initialize wheel directions
int left_wheel_direction = CCW;
int right_wheel_direction = CW;

int* pi_int;

// Read wheel encoder values
void read_encoder_values(int *left_encoder_value, int *right_encoder_value)
{
    *left_encoder_value = left_wheel_pulse_count;
    *right_encoder_value = right_wheel_pulse_count;
}

// Left wheel callback function
void left_wheel_pulse(int pi, u_int user_gpio, u_int level, uint32_t tick)
{   
    (void)user_gpio;
    (void)level;
    (void)tick;
    // left wheel direction 
    // CCW - forward
    // CW - backward
    
    // Read encoder direction value for left wheel
    left_wheel_direction = gpio_read(pi,LEFT_DIRECTION_PIN); 

    if(left_wheel_direction == CCW)
        left_wheel_pulse_count++;
    else 
        left_wheel_pulse_count--;
}

// Right wheel callback function
void right_wheel_pulse(int pi, u_int user_gpio, u_int level, uint32_t tick)
{
    (void)user_gpio;
    (void)level;
    (void)tick;
    // right wheel direction 
    // CW - forward,  
    // CCW - backward

    // Read encoder direction value for right wheel
    right_wheel_direction = gpio_read(pi,RIGHT_DIRECTION_PIN); 

    if(right_wheel_direction = CW)
        right_wheel_pulse_count++;
    else 
        right_wheel_pulse_count--;
}

// Set each motor speed from the respective velocity command interface
void set_motor_speeds(double left_wheel_command, double right_wheel_command)
{   
    
    // Tune motor speeds by adjusting the command coefficients. These are dependent on the number of encoder ticks. 3000 ticks and above work well with coefficients of 1.0
    double left_motor_speed = ceil(left_wheel_command * 1.65);
    double right_motor_speed = ceil(right_wheel_command * 1.65);

    // Set motor directions
    if(left_motor_speed > 0) 
        gpio_write(*pi_int,LEFT_DIRECTION_PIN, CCW);
    else
        gpio_write(*pi_int,LEFT_DIRECTION_PIN, CW);

    if(right_motor_speed > 0)
        gpio_write(*pi_int,RIGHT_DIRECTION_PIN, CW);
    else
       gpio_write(*pi_int,RIGHT_DIRECTION_PIN, CCW);

    set_PWM_dutycycle(*pi_int,LEFT_PWM_PIN,(int) abs(left_motor_speed));
    set_PWM_dutycycle(*pi_int,RIGHT_PWM_PIN,(int) abs(left_motor_speed));

}

void handler(int signo)
{
    set_PWM_dutycycle(*pi_int,LEFT_PWM_PIN,0);
    set_PWM_dutycycle(*pi_int,RIGHT_PWM_PIN,0);

    exit(0);
}