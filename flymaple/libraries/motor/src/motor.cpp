#include "motor/motor.h"

using namespace crim;

Motor::Motor(uint8_t pwm_pin, uint8_t dir_1_pin, uint8_t dir_2_pin, uint64_t frequency)
    : pwm_pin_(pwm_pin), dir_1_pin_(dir_1_pin), dir_2_pin_(dir_2_pin), frequency_(frequency) {
  //
  pinMode(pwm_pin_, PWM);
  pinMode(dir_1_pin_, OUTPUT);
  pinMode(dir_2_pin_, OUTPUT);
  
  // 
  uint32 period;// in microseconds
  period = (1000000/frequency);// "period = (1/frequency)*1000000;" does _not_ work
  
	Timer3.setPeriod(period);
}                               

Motor::~Motor()
{
  // nothing
}

void Motor::forward(uint8_t duty_cycle) {
  digitalWrite(dir_1_pin_, HIGH);
  digitalWrite(dir_2_pin_, LOW);
  
  set_duty_cycle(duty_cycle);
}

void Motor::backward(uint8_t duty_cycle) {
  digitalWrite(dir_1_pin_, LOW);
  digitalWrite(dir_2_pin_, HIGH);
  
  set_duty_cycle(duty_cycle);
}

void Motor::stop(uint8_t type) {
  switch(type) {
    case 0: {
      // Brake to GND
      digitalWrite(dir_1_pin_, LOW);
      digitalWrite(dir_2_pin_, LOW);
      break;
    }
    case 1: {
      // Brake to VCC
      digitalWrite(dir_1_pin_, HIGH);
      digitalWrite(dir_2_pin_, HIGH);
      break;
    }
  }
  // Brake to VCC
  digitalWrite(dir_1_pin_, HIGH);
  digitalWrite(dir_2_pin_, HIGH);
}

void Motor::set_duty_cycle(uint8_t duty_cycle) {
  if (duty_cycle > 100) duty_cycle = 100;
  
  //"pwmWrite(pwm_pin_, Timer3.getOverflow()*(duty_cycle/100));" does _not_ work
  pwmWrite(pwm_pin_, (Timer3.getOverflow()/100)*duty_cycle);
  
}
