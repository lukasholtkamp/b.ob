#include <iostream> 
#include "MotorEncoder.hpp"

/**
 * @brief Construct a new ENC::Encoder::Encoder object
 * 
 * @param gpio
 * @param callback 
 */

namespace ENC{
	
Encoder::Encoder(int pi_var,int gpio, encoderCB_t callback, int tpr){
  pi = pi_var;
  e_Pin = gpio;
  mycallback = callback;

  ticks_per_rev = tpr;

  _weighting=0.1;
  _new=1.0-_weighting;
  _old=_weighting;

  _high_tick = 0;
  _period = 0;
  
  // set pins to input mode with a pullup restor
  set_mode(pi,gpio,PI_INPUT);
  set_pull_up_down(pi,gpio,PI_PUD_UP);

  // set alert function for changes in the signal
  callback_ex(pi,gpio,EITHER_EDGE,_pulseEx,this);
  
}

/** 
 * @brief Cancels the reader and releases resources.
 */
void Encoder::re_cancel(void){
  callback_ex(pi,e_Pin,EITHER_EDGE,0,this);
}

/** @brief alert function
* @param gpio pin which changed state
* @param level 0 = change to low (a falling edge) 1 = change to high (a rising edge) 2 = no level change (a watchdog timeout)
* @param tick The number of microseconds since boot WARNING: this wraps around from 4294967295 to 0 roughly every 72 minutes
*/
void Encoder::_pulse(int pi, u_int gpio, u_int level, uint32_t tick){

  (void)pi;
  (void)gpio;
  // time between pulses
  int32_t t;

  // rising edge
  if(level == 1){

    if(_high_tick != 0){
        // find period between last pulse and this pulse
        t = _tick_diff(_high_tick,tick);

        if(_period != 0){
          // update in a smoothed fashion
          _period = u_int32_t((_old*double(_period))+(_new*double(t)));

          // return 1 pulse to the callback function
          (mycallback)(1);
        }
        else{
          _period = t;
        }

        mperiod.insert(_period);
        med_period = mperiod.getMedian();
    }
    _high_tick = tick;
  }

}

/** 
 * @brief alert function
 * @param gpio 
 * @param level 
 * @param tick 
 * @param user 
 */
void Encoder::_pulseEx(int pi, u_int gpio, u_int level, u_int32_t tick, void *user){

  Encoder *mySelf = (Encoder *) user;

  mySelf->_pulse(pi,gpio,level,tick);

}

/** 
 * @brief calculate difference between two times
 * @param o_tick old tick
 * @param c_tick current tick
 * @return time difference in microseconds
 */
u_int32_t Encoder::_tick_diff(u_int32_t o_tick, u_int32_t c_tick){
  return c_tick-o_tick;
}

/**
 * @brief calculate frequency
 * @return frequency in Hz
 */
double Encoder::getFreq() const{
  if(_period != 0){
    return 1000000.0/ med_period;
  }
  else{
    return 0.0;
  }

}

/**
 * @brief get RPM of motor
 * @return RPM
 * @details Formula found in BLDC-8015A manual and the number of motor pole pairs found at https://docs.odriverobotics.com/v/0.5.4/hoverboard.html
 */
double Encoder::getMotorSpeed() const{
  return (60*getFreq())/double(ticks_per_rev);
}

/**
 * @brief get Period
 * @return period in microseconds
 */ 
u_int32_t Encoder::getPeriod() const{
  return med_period;
}


}