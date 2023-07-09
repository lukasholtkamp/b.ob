#include "motor_driver_library/MotorDriver.h"

//#include <JetsonGPIO.h> //<-- Used to control the GPIO pins on the Jetson Nano
#include <softPwm.h> //<-- Used to control the PWM pins on the Raspberry Pi
#include <wiringPi.h> //<-- Used to control the GPIO pins on the Raspberry Pi

namespace MD {
  /**
   * @brief Construct a new MD::Motor::Motor object
   * @param direction_pin 
   * @param pwm_pin 
   * @param min_speed 
   * @param max_speed 
   */
  Motor::Motor(int direction_pin, int pwm_pin, double min_speed, double max_speed) {
    direction_pin_ = direction_pin;
    pwm_pin_ = pwm_pin;
    min_speed_ = min_speed;
    max_speed_ = max_speed;
    speed_ = 0.0;
    direction_ = 0;

    wiringPiSetup();
    // Pin configurations for JetsonGPIO
    //GPIO::setmode(GPIO::BOARD); // GPIO::BCM or GPIO::BOARD
    //GPIO::setup(32, GPIO::OUT); // Left PWM
    //GPIO::setup(31, GPIO::OUT); // Left direction
    //GPIO::setup(33, GPIO::OUT); // Right PWM
    //GPIO::setup(35, GPIO::OUT); // Right direction

    // --- Left Motor ---
    pinMode(left_pwm_pin, OUTPUT);		// Left PWM
    softPwmCreate(left_pwm_pin,0,100);
    pinMode(left_direction_pin, OUTPUT); 		// Left direction
    // --- Right Motor ---
    pinMode(right_pwm_pin, OUTPUT);		// Right PWM
    softPwmCreate(right_pwm_pin,0,100);
    pinMode(right_direction_pin, OUTPUT); 		// Right direction
  } 

  /** @brief Returns the pwm pin of the motor
   * @return int
   */ 
  int MD::Motor::GetPwmPin() {
    return pwm_pin_;
  }

  /**
   * @brief Set the direction of the object
   * @param speed 
   * @param pin_number 
   * @details This function is used to set the direction of the motor. If the speed is positive, the direction is forward. If the speed is negative, the direction is backward.
   */
  void MD::Motor::SetDirection(double speed, int direction_pin) {
    if (speed > min_speed_) {
      direction_ = true;
      digitalWrite(direction_pin, HIGH);
    } else if (speed <= min_speed_) {
      direction_ = false;
      digitalWrite(direction_pin, LOW);
    }
  }

  /**
   * @brief Get the direction of the object
   * @return true 
   * @return false 
   */
  bool MD::Motor::GetDirection() {
    return direction_;
  }

  /// @brief Returns the direction pin of the motor
  int MD::Motor::GetDirectionPin() {
    return direction_pin_;
  }

  /**
   * @brief Set the Speed of the object
   * @param speed 
   */
  void MD::Motor::SetSpeed(double speed) {
    if (speed < min_speed_ && speed > max_speed_ * -1) { // If the speed is less than the minimum speed and greater than the maximum speed, the speed is set to the minimum speed
        speed_ = speed * -1;
    } else if (speed <= max_speed_ * -1) { // If the speed is less than the maximum speed, the speed is set to the maximum speed
      speed_ = max_speed_;
    } else if (speed >= max_speed_) {  // If the speed is greater than the maximum speed, the speed is set to the maximum speed
        speed_ = max_speed_;
    } else {                        // If the speed is greater than the minimum speed and less than the maximum speed, the speed is set to the speed
      speed_ = speed;
    }
  }

  /**
   * @brief Get the Speed of the object
   * @return double 
   */
  double MD::Motor::GetSpeed() {
    return speed_;
  }

  /**
   * @brief Set the Min Speed of the object
   * @return double 
   */
  void MD::Motor::SetMinSpeed(double min_speed) {
    min_speed_ = min_speed;
  }

  /**
   * @brief Set the Max Speed of the object
   * @return double 
   */
  void MD::Motor::SetMaxSpeed(double max_speed) {
    max_speed_ = max_speed;
  }

  /**
   * @brief Stop the motor
   */
  void MD::Motor::Stop() {
    speed_ = 0.0;
  }
} // namespace MD