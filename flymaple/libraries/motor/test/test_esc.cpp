#include "wirish.h"
#include "motor/motor.h"

using namespace crim;

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
  // Set up motors
  uint8_t motor_pwm_pin = D12;
  uint64_t frequency = 50;

  crim::Esc bldc_motor(motor_pwm_pin, frequency);
  
  // The loop    
  while (true) {
    
    bldc_motor.set_speed(0);
    delay(3000);
    
    for (uint8_t i=1; i<4; ++i) {
      uint8_t speed;
      speed = i*20;
      
      bldc_motor.set_speed(speed);
      delay(2000);
    }
    
    for (uint8_t i=4; i>1; ++i) {
      uint8_t speed;
      speed = i*20;
      
      bldc_motor.set_speed(speed);
      delay(2000);
    }
  }

  return 0;
}
