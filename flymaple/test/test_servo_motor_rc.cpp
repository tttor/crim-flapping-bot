// Unit test: Servo_ control with RC 
// @author vektor dewanto
#include <vector>
#include <stdint.h> // for uint16_t, uint8_t, etc
#include <string>
#include "wirish.h"
#include "rc/radio_control.h"
#include "servo/maestro_servo.h"
#include "motor/motor.h"

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
	init();
}

int main(void) {
  using namespace crim;
  using namespace std;
  
  //////////////////////////////////////////////////////////////////////////////
  // Setting up the RC
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
  
  // Wait for RC to be at its normal position
  // TODO make this in the RC class (?)    
  uint16_t ch_1_normal_ppm = 1504;
  uint16_t ch_2_normal_ppm = 1504;
  uint16_t ch_3_normal_ppm = 1092;
  uint16_t ch_4_normal_ppm = 1504;
  
  while((rc.read(1) < (ch_1_normal_ppm-RadioControl::kPPMTolerance)) || (rc.read(1) > (ch_1_normal_ppm+RadioControl::kPPMTolerance))) continue;
  while((rc.read(2) < (ch_2_normal_ppm-RadioControl::kPPMTolerance)) || (rc.read(2) > (ch_2_normal_ppm+RadioControl::kPPMTolerance))) continue;
  while((rc.read(3) < (ch_3_normal_ppm-RadioControl::kPPMTolerance)) || (rc.read(3) > (ch_3_normal_ppm+RadioControl::kPPMTolerance))) continue;
  while((rc.read(4) < (ch_4_normal_ppm-RadioControl::kPPMTolerance)) || (rc.read(4) > (ch_4_normal_ppm+RadioControl::kPPMTolerance))) continue;
  
  uint16_t ch_1_PPM_init = rc.read(1);
  uint16_t ch_2_PPM_init = rc.read(2);
  uint16_t ch_3_PPM_init = rc.read(3);
  uint16_t ch_4_PPM_init = rc.read(4);
    
  const double half_range_x = (RadioControl::kPPMThresholdAbove - RadioControl::kPPMThresholdBelow) * 0.5;
  const double half_range_y = half_range_x;  
  const double half_ch_4_range = (RadioControl::kPPMThresholdAbove - RadioControl::kPPMThresholdBelow) * 0.5;
  const double ch_3_range = (RadioControl::kPPMThresholdAbove - RadioControl::kPPMThresholdBelow);
  
  //////////////////////////////////////////////////////////////////////////////
  // Setting up the servos
  uint16_t servo_1_min_pos = 70;
  uint16_t servo_1_max_pos = 180;
  
  MaestroServo servo_1(0, servo_1_min_pos, servo_1_max_pos);
  servo_1.star_poses["normal_pos"] = (servo_1_max_pos+servo_1_min_pos)/2;
  servo_1.star_poses["straight_front"] = 90;
  
  uint16_t servo_2_min_pos = 90;
  uint16_t servo_2_max_pos = 180;
  
  MaestroServo servo_2(1, servo_2_min_pos, servo_2_max_pos);
  servo_2.star_poses["fully_open"] = servo_2_max_pos;
  servo_2.star_poses["fully_closed"] = servo_2_min_pos;
  
  //////////////////////////////////////////////////////////////////////////////
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
  
  //////////////////////////////////////////////////////////////////////////////
  while (true) {
    SerialUSB.println("looping...");
    
    ////////////////////////////////////////////////////////////////////////////
    // Motor control
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
    
    ////////////////////////////////////////////////////////////////////////////
    // Servo control
    uint16_t pos;
    int32_t delta_pos;
    
    // Servo_1
    int32_t ch_4;
    ch_4 = rc.read(4) - ch_4_PPM_init;
    //SerialUSB.println(ch_4, DEC);
    
    if (ch_4 > 0) {
      delta_pos = ((double)ch_4/half_ch_4_range) * (servo_1.max_pos() - servo_1.star_poses["normal_pos"]);
    } else {
      delta_pos = ((double)ch_4/half_ch_4_range) * (servo_1.star_poses["normal_pos"] - servo_1.min_pos());
    }
    
    pos = servo_1.star_poses["normal_pos"];
    pos += delta_pos;
    
    //SerialUSB.println(pos, DEC);
    servo_1.go_to(pos);
    
    // Servo_2
    uint16_t ch_3;
    ch_3 = rc.read(3) - ch_3_PPM_init;
    //SerialUSB.println(ch_3, DEC);
    
    delta_pos = ((double)ch_3/ch_3_range) * (servo_2.max_pos()-servo_2.min_pos());
    
    pos = servo_2.star_poses["fully_closed"];
    pos += delta_pos;
    
    //SerialUSB.println(pos, DEC);
    servo_2.go_to(pos);
    
    ////////////////////////////////////////////////////////////////////////////
    delay(100);
  } 
  
  return 0;
}
