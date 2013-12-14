#include "wirish.h"
#include "motor/motor.h"

using namespace crim;

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
  // Set up motors
  uint8_t r_motor_pwm_pin = D12;
  uint8_t r_motor_dir_1_pin = D14;
  uint8_t r_motor_dir_2_pin = D24;
  uint64_t r_frequency = 50000;

  crim::Motor right_motor(r_motor_pwm_pin, r_motor_dir_1_pin, r_motor_dir_2_pin, r_frequency);
  
  uint8_t l_motor_pwm_pin = D28;
  uint8_t l_motor_dir_1_pin = D27;
  uint8_t l_motor_dir_2_pin = D11;
  uint64_t l_frequency = r_frequency;
  
  crim::Motor left_motor(l_motor_pwm_pin, l_motor_dir_1_pin, l_motor_dir_2_pin, l_frequency);
  
  // The loop    
  while (true) {
    SerialUSB.println("motor.forward()");
    for (uint8_t i=1; i<5; ++i) {
      uint8_t duty_cycle;
      duty_cycle = i*20;
      
      left_motor.forward(duty_cycle);
      right_motor.forward(duty_cycle);
      delay(1500);
    }
    
    SerialUSB.println("motor.stop()");
    left_motor.stop();
    right_motor.stop();
    delay(3000);
    
    SerialUSB.println("motor.backward()");
    for (uint8_t i=1; i<5; ++i) {
      uint8_t duty_cycle;
      duty_cycle = i*20;
      
      left_motor.backward(duty_cycle);
      right_motor.backward(duty_cycle);
      delay(1500);
    }
    
    SerialUSB.println("motor.stop()");
    left_motor.stop();
    right_motor.stop();
    delay(3000);
  }

  return 0;
}
