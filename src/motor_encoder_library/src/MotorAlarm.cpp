#include <iostream>                                                                                                                                                                                                                                                                                                                                                                                                                                                 MotorDriver.cpp                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
#include "MotorAlarm.hpp"


namespace ALM{
	
Alarm::Alarm(int gpio, alarmCB_t callback){
  a_Pin = gpio;
  mycallback = callback;
  
  gpioSetMode(gpio,PI_INPUT);
  gpioSetPullUpDown(gpio,PI_PUD_UP);
  gpioSetAlertFuncEx(gpio,_pulseEx,this);

  state = gpioRead(gpio);
  
}

/// @brief Returns the pwm pin of the motor
/// @return int
void Alarm::re_cancel(void){
  gpioSetAlertFuncEx(a_Pin,0,this);
}

void Alarm::_pulse(int gpio, int level,uint32_t tick){
    state = gpioRead(a_Pin);
}

void Alarm::_pulseEx(int gpio, int level,u_int32_t tick, void *user){

  Alarm *mySelf = (Alarm *) user;

  mySelf->_pulse(gpio,level,tick);

}

int Alarm::getState(){
    return state;
}


}
