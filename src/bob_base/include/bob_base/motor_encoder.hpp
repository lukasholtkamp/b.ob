#ifdef __cplusplus
extern "C"
{
#endif

#ifndef __MOTOR_ENCODER_H__
#define __MOTOR_ENCODER_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <pigpiod_if2.h> //<-- Used to read the gpio on the Raspberry Pi

#define LEFT_ALARM_PIN 22    //<-- Pin number for the direction of the left motor
#define RIGHT_ALARM_PIN 25   //<-- Pin number for the pwm of the left motor
#define LEFT_ENCODER_PIN 23  //<-- Pin number for the left motor encoder
#define RIGHT_ENCODER_PIN 24 //<-- Pin number for the right motor encoder

#define LEFT_DIRECTION_PIN 6   //<-- Pin number for the direction of the left motor
#define RIGHT_DIRECTION_PIN 19 //<-- Pin number for the direction of the right motor
#define LEFT_PWM_PIN 13        //<-- Pin number for the pwm of the left motor
#define RIGHT_PWM_PIN 12       //<-- Pin number for the pwm of the right motor

#define CCW 1 //<-- Value to be written to direction pin for motor to go counterclockwise
#define CW 0  //<-- Value to be written to direction pin for motor to go clockwise

    void handler(int signo);
    void left_wheel_pulse(int pi, u_int user_gpio, u_int level, uint32_t tick);
    void right_wheel_pulse(int pi, u_int user_gpio, u_int level, uint32_t tick);
    void set_motor_speeds(int pi, double left_wheel_command, double right_wheel_command);
    void read_encoder_values(int *left_encoder_value, int *right_encoder_value);

    extern int left_wheel_pulse_count;
    extern int right_wheel_pulse_count;
    extern int left_wheel_direction;
    extern int right_wheel_direction;

#endif

#ifdef __cplusplus
}
#endif