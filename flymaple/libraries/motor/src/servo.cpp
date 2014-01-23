#include "motor/servo.h"

using namespace crim;

Servo::Servo(uint8_t pwm_pin)
  : pwm_pin_(pwm_pin) {
  
  uint32 period;
  
  pinMode(pwm_pin_, PWM);
  
  period = 20000;// "period in uS
  
	Timer3.setPeriod(period);
}                               

Servo::~Servo()
{
  // nothing
}

void Servo::set_angle(uint8_t angle) {  // speed : 0 - 100
  uint16_t pwm;
  
  if (angle > 180) angle = 180;
  else if(angle <= 0 ) angle = 0;
  
  // MIN_ANGLE : Timer3.getOverflow()*1000/20000
  // MAX ANGLE : Timer3.getOverflow()*2000/20000
  
  pwm = map(angle,0,180, Timer3.getOverflow()*700/20000, Timer3.getOverflow()*2300/20000);
  
  pwmWrite(pwm_pin_, pwm);
  
}
