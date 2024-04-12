//~ # GNU nano 4.8      
#include <iostream>                                                                                                                                                                                                                                                                                                                                                                                                                                                 MotorDriver.cpp                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
#include "MotorEncoder.hpp"

/**
 * @brief Construct a new MD::Motor::Motor object
 * 
 * @param directionPin 
 * @param pwmPin 
 * @param minSpeed 
 * @param maxSpeed 
 */

namespace ENC{
	
Encoder::Encoder(int gpio){
  e_Pin = gpio;

  _weighting=0;
  _new=1.0-_weighting;
  _old=_weighting;

  _high_tick = NULL;
  _period = NULL;
  _high = NULL;
  
  gpioSetMode(gpio,PI_INPUT);
  gpioSetPullUpDown(gpio,PI_PUD_UP);
  gpioSetAlertFuncEx(gpio,_pulseEx,this);
  
}

/// @brief Returns the pwm pin of the motor
/// @return int
void Encoder::re_cancel(void){
  gpioSetAlertFuncEx(e_Pin,0,this);
}

void Encoder::_pulse(int gpio, int level,uint32_t tick){

  int32_t t;

  if(level == 1){

    if(_high_tick != NULL){
        t = _tick_diff(_high_tick,tick);

        if(_period != NULL){
          _period = (_old*_period)+(_new*t);
        }
        else{
          _period = t;
        }
    }
    _high_tick = tick;
  }

  else if(level == 0){

    if(_high_tick != NULL){
        t = _tick_diff(_high_tick,tick);

        if(_high != NULL){
          _high = (_old*_high)+(_new*t);
        }
        else{
          _high= t;
        }
    }
  }

}

void Encoder::_pulseEx(int gpio, int level,u_int32_t tick, void *user){

  Encoder *mySelf = (Encoder *) user;

  mySelf->_pulse(gpio,level,tick);

}

u_int32_t Encoder::_tick_diff(u_int32_t o_tick, u_int32_t c_tick){
  return c_tick-o_tick;
}

double Encoder::getFreq() const{
  if(_period != NULL){
    return 1000000.0/_period;
  }
  else{
    return 0.0;
  }

}

double Encoder::getMotorSpeed() const{
  return (60*getFreq())/(15*6);
}

}
