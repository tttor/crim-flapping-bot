#include "motor/esc.h"

using namespace crim;

Esc::Esc(uint8_t pwm_pin, uint64_t frequency)
  : pwm_pin_(pwm_pin), frequency_(frequency) {
  //
  pinMode(pwm_pin_, PWM);
  
  // 
  uint32 period;// in microseconds
  period = (1000000/frequency);// "period = (1/frequency)*1000000;" does _not_ work
  
	Timer3.setPeriod(period);
}                               

Esc::~Esc()
{
  // nothing
}

void Esc::set_speed(uint8_t speed) {  // speed : 0 - 100
  if (speed > 100) speed = 100;
  else if(speed <= 0 ) speed = 0;
  
  pwmWrite(pwm_pin_, Timer3.getOverflow()*(1100 + speed*10)/20000);
  
  /*
   * period_cycle = 72*period (uS)
   * prescaler = period_cycle/65535 + 1
   * overflow = period_cycle/prescaler
   * 
   * low speed = 1000 uS -> 1000 uS / 20000 uS * overflow
   * high speed = 2000 uS -> 2000 uS / 20000 uS * overflow
   * 
   *
   */
  
}
