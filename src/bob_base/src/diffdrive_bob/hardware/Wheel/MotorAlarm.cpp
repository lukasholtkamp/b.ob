#include <iostream>
#include "MotorAlarm.hpp"

/**
 * @brief Construct a new ALM::Alarm::Alarm object
 * 
 * @param gpio
 */

namespace ALM{
	
Alarm::Alarm(int pi_val,int gpio){
  pi = pi_val;
  a_Pin = gpio;
  
  // set pins to input mode with a pullup restor
  set_mode(pi,gpio,PI_INPUT);
  set_pull_up_down(pi,gpio,PI_PUD_DOWN);
  
  // set alert function for changes in the signal
  callback_ex(pi,gpio,EITHER_EDGE,_pulseEx,this);

  // read inital state
  state = gpio_read(pi,gpio);
  
}

/** 
 * @brief Cancels the reader and releases resources.
 */
void Alarm::re_cancel(void){
  callback_ex(pi,a_Pin,EITHER_EDGE,0,this);
}

/** @brief alert function
* @param gpio pin which changed state
* @param level 0 = change to low (a falling edge) 1 = change to high (a rising edge) 2 = no level change (a watchdog timeout)
* @param tick The number of microseconds since boot WARNING: this wraps around from 4294967295 to 0 roughly every 72 minutes
*/
void Alarm::_pulse(int pi, u_int gpio, u_int level,uint32_t tick){
  (void)gpio;
  (void)level;
  (void)tick;
  // if there is a change in state fo the pin, update the state
  state = gpio_read(pi,a_Pin);
}


/** 
 * @brief alert function
 * @param gpio 
 * @param level 
 * @param tick 
 * @param user 
 */
void Alarm::_pulseEx(int pi, u_int gpio, u_int level,u_int32_t tick, void *user){

  Alarm *mySelf = (Alarm *) user;

  mySelf->_pulse(pi,gpio,level,tick);

}

/**
 * @brief get State
 * @return (1 = no alarm, 0 = alarm)
 */ 
int Alarm::getState() const{
    return state;
}


}