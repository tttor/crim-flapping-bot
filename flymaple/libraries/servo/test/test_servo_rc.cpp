// Unit test: Servo_ control with RC 
// @author vektor dewanto
#include <vector>
#include <stdint.h> // for uint16_t, uint8_t, etc
#include <string>
#include "wirish.h"
#include "rc/radio_control.h"
#include "servo/maestro_servo.h"

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
	init();
}

int main(void) {
  using namespace crim;
  using namespace std;
  
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
  uint16_t ch_3_normal_ppm = 1092;
  uint16_t ch_4_normal_ppm = 1504;
  
  while((rc.read(3) < (ch_3_normal_ppm-RadioControl::kPPMTolerance)) || (rc.read(3) > (ch_3_normal_ppm+RadioControl::kPPMTolerance))) continue;
  while((rc.read(4) < (ch_4_normal_ppm-RadioControl::kPPMTolerance)) || (rc.read(4) > (ch_4_normal_ppm+RadioControl::kPPMTolerance))) continue;
  
  //uint16_t ch_3_PPM_init = rc.read(3);
  uint16_t ch_4_PPM_init = rc.read(4);
  
  double half_ch_4_range = (RadioControl::kPPMThresholdAbove - RadioControl::kPPMThresholdBelow) * 0.5;
  
  // Setting up the servos
  uint16_t servo_1_min_pos = 70;
  uint16_t servo_1_max_pos = 180;
  
  MaestroServo servo_1(0, servo_1_min_pos, servo_1_max_pos);
  servo_1.star_poses["normal_pos"] = (servo_1_max_pos+servo_1_min_pos)/2;
  servo_1.star_poses["straight_front"] = 90;
   
  //
  while (true) {
    SerialUSB.println("looping...");
    
    int32_t ch_4;
    ch_4 = rc.read(4) - ch_4_PPM_init;
    SerialUSB.println(ch_4, DEC);
    
    int32_t delta_pos;
    if (ch_4 > 0) {
      delta_pos = ((double)ch_4/half_ch_4_range) * (servo_1.max_pos() - servo_1.star_poses["normal_pos"]);
    } else {
      delta_pos = ((double)ch_4/half_ch_4_range) * (servo_1.star_poses["normal_pos"] - servo_1.min_pos());
    }
    
    uint16_t pos = servo_1.star_poses["normal_pos"];
    pos += delta_pos;
    
    SerialUSB.println(pos, DEC);
    servo_1.go_to(pos);
    
    delay(300);
  } 
  
  return 0;
}
