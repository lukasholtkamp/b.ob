/**
 * @file MotorAlarm.hpp
 * @brief Motor Alarm library for the Raspberry Pi
 * @details This library is used to check the alarms on the motors of the robot.
 */
#ifndef MOTOR_ALARM_HPP
#define MOTOR_ALARM_HPP

#include <pigpio.h> //<-- Used to read the gpio on the Raspberry Pi

#define LEFT_ALARM_PIN      22 //<-- Pin number for the direction of the left motor
#define RIGHT_ALARM_PIN     25 //<-- Pin number for the pwm of the left motor


// ALM: Motor Alarm
namespace ALM{
  /**
   * @brief Alarm class
   * @details This class is to used to read when the motor alarm is triggered.
   */
  class Alarm {

  private:
    int a_Pin = 0; //<-- alarm Pin numbers
    int state = 0; //<-- state of alarm pin (1 = no alarm, 0 = alarm)

    void _pulse(int gpio, int level, uint32_t tick); //<-- function gets called everytime there is a change on the a_Pin

    static void _pulseEx(int gpio, int level, uint32_t tick, void *user);//<-- function gets called everytime there is a change on the a_Pin

  public:
  /**
   * @brief Constructor
   * @details This constructor is used to initialize the motor alarm. The constructor is used to set the pin numbers for the motor alarms.
   * @param gpio Pin number for the alarm signal from the motor
   */
    Alarm() = default;
    
    Alarm(int gpio);

    void re_cancel(void); //<--Cancels the reader and releases resources.

    int getState() const; //<--Gets state of the alarm

  };
  }

#endif // MOTOR_ALARM_HPP