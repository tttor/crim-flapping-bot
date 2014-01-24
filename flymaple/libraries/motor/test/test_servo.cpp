#include "wirish.h"
#include "motor/servo.h"

using namespace crim;

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
  // Set up motors
  uint8_t motor_pwm_pin = D12;

  crim::Servo servo(motor_pwm_pin);
  
  // The loop    
  while (true) {
    servo.set_angle(0);
    delay(2000);
    
    servo.set_angle(90);
    delay(2000);
    
    servo.set_angle(180);
    delay(2000);
    
    servo.set_angle(90);
    delay(2000);
    
  }

  return 0;
}
