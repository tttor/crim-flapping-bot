#include "electronics_speed_controller.hpp"

using namespace crim;

ElectronicsSpeedController::ElectronicsSpeedController(uint8_t pwm_pin)
    : pwm_pin_(pwm_pin), min_throtle_(1100), max_throtle_(1900) {
  //        
}

ElectronicsSpeedController::~ElectronicsSpeedController() {
  //
}

void ElectronicsSpeedController::set_duty_cycle(double duty_cycle) {
  if (duty_cycle > 100.) duty_cycle = 100.;
  if (duty_cycle < .0) duty_cycle = 0.;

  const uint8_t duty_cycle_base = 125;// required by the ESC; minimal duty cycle
  const uint8_t duty_cycle_max = 255;// see http://arduino.cc/en/Reference/AnalogWrite#.UwL-td_LXCQ
  const uint8_t duty_cycle_range = duty_cycle_max - duty_cycle_base;
    
  // Normalize to the range of 0 to (255-duty_cycle_base=130)
  uint8_t norm_duty_cycle;
  norm_duty_cycle = (duty_cycle/100) * duty_cycle_range;// divided by 100 because duty_cycle is in percentage
  
  // Assign 
  analogWrite(pwm_pin_, duty_cycle_base + norm_duty_cycle);
}

void ElectronicsSpeedController::set_duty_cycle(uint16_t throtle) {
  uint16_t throtle_range;
  throtle_range = max_throtle_ - min_throtle_;
  
  int32_t throtle_delta;
  throtle_delta = (int32_t)throtle - (int32_t)min_throtle_;
  
  if (throtle_delta < 0) throtle_delta = 0;
  //Serial.print("throtle_delta= "); Serial.println((long int)throtle_delta);
  
  double duty_cycle;
  duty_cycle = (double) throtle_delta / throtle_range * 100.;// times 100 as the duty_cycle is in percentage
  //Serial.print("duty_cycle= "); Serial.println(duty_cycle);
  
  set_duty_cycle(duty_cycle);
}

void ElectronicsSpeedController::set_throtle_range(uint16_t min, uint16_t max) {
  if (min > max) return;
  
  min_throtle_ = min;
  max_throtle_ = max;
}
