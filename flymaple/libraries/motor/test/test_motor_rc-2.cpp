// @author vektor dewanto
#include <vector>
#include <stdint.h> // for uint16_t, uint8_t, etc
#include <string>

#include "wirish.h"
#include "motor/motor.h"
#include "rc/radio_control.h"

using namespace crim;
using namespace std;

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
  // Set up motors
  unsigned char r_motor_pwm_pin = D12;
  unsigned char r_motor_dir_1_pin = D24;
  unsigned char r_motor_dir_2_pin = D14;
  unsigned long r_frequency = 50000;

  crim::Motor right_motor(r_motor_pwm_pin, r_motor_dir_1_pin, r_motor_dir_2_pin, r_frequency);
  
  unsigned char l_motor_pwm_pin = D28;
  unsigned char l_motor_dir_1_pin = D27;
  unsigned char l_motor_dir_2_pin = D11;
  unsigned long l_frequency = r_frequency;
  
  crim::Motor left_motor(l_motor_pwm_pin, l_motor_dir_1_pin, l_motor_dir_2_pin, l_frequency);
  
  // Set up RC
  vector<uint8_t> rc_ch_pins;
  rc_ch_pins.resize(8);
  rc_ch_pins.at(0) = D31;
  rc_ch_pins.at(1) = D32;
  rc_ch_pins.at(2) = D33;
  rc_ch_pins.at(3) = D34;
  rc_ch_pins.at(4) = D35;
  rc_ch_pins.at(5) = D36;
  rc_ch_pins.at(6) = D37;
  rc_ch_pins.at(7) = D26;
  
  RadioControl rc(rc_ch_pins);
  
  // Wait for RC to be ready
  // TODO make this in the RC class (?)    
  while(rc.read(1) < 1500 || rc.read(1) > 1550) continue;
  while(rc.read(2) < 1500 || rc.read(2) > 1550) continue;
  
  uint16_t ch_1_PPM_init = rc.read(1);
  uint16_t ch_2_PPM_init = rc.read(2);
  
  const uint16_t half_range_x = (RadioControl::kPPMThresholdAbove - RadioControl::kPPMThresholdBelow) * 0.5;
  const uint16_t half_range_y = half_range_x;       
  
  // The loop    
  while (true) {
    int32_t x;
    x = rc.read(1) - ch_1_PPM_init;
    
    int32_t y;
    y = rc.read(2) - ch_2_PPM_init;
  
    uint8_t pwm;
    pwm = abs( ((double)y/half_range_y) * 100 );
    
    uint8_t add_pwm;
    add_pwm = abs( ((double)x/half_range_x) * 100 );
    
    if(y >= 0) {// move forward
      if(x >= 0) {// left motors win
        left_motor.forward(pwm+add_pwm);
        right_motor.forward(pwm);
      } else {
        left_motor.forward(pwm);
        right_motor.forward(pwm+add_pwm);
      }
    } else {// move backward
      if(x >= 0) {// left motors win
        left_motor.backward(pwm+add_pwm);
        right_motor.backward(pwm);
      } else {
        left_motor.backward(pwm);
        right_motor.backward(pwm+add_pwm);
      }
    }
  
    delay(50);
  }

  return 0;
}
