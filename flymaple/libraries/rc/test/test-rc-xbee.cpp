// Unit test: RC 
#include <vector>
#include <stdint.h> // for uint16_t, uint8_t, etc
#include <string>
#include "wirish.h"

#include "rc/radio_control.h"
#include "mavlink/v1.0/common/mavlink.h"
#include "xbee/flymaple_mavlink_packet_handler.h"

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
	init();
}

int main(void) {
  using namespace crim;
  using namespace std;
  
  vector<uint8_t> rc_ch_pins;
  rc_ch_pins.resize(8);
  rc_ch_pins.at(0) = D31;
  rc_ch_pins.at(1) = D32;
  rc_ch_pins.at(2) = D33;
  //rc_ch_pins.at(3) = D34;
  //rc_ch_pins.at(4) = D35;
  //rc_ch_pins.at(5) = D36;
  //rc_ch_pins.at(6) = D37;
  //rc_ch_pins.at(7) = D26;
  
  RadioControl rc(rc_ch_pins);
  
  //// Wait for RC to be at its normal position
  //// TODO make this in the RC class (?)    
  //uint16_t ch_1_normal_ppm = 1496;// TGY 9X: 1496; Futaba F8TG: 1508 
  //uint16_t ch_2_normal_ppm = 1455;// TGY 9X: 1455; Futaba F8TG: 1510
  
  //pinMode(BOARD_LED_PIN, OUTPUT);
  //while( (rc.read(1) < (ch_1_normal_ppm-RadioControl::kPPMTolerance)) or 
         //(rc.read(1) > (ch_1_normal_ppm+RadioControl::kPPMTolerance)) or
         //(rc.read(2) < (ch_2_normal_ppm-RadioControl::kPPMTolerance)) or
         //(rc.read(2) > (ch_2_normal_ppm+RadioControl::kPPMTolerance))
       //) {
    //togglePin(BOARD_LED_PIN);
    //delay(100);
  //}
  //digitalWrite(BOARD_LED_PIN, LOW);
  
  //
  mavlink_system_t mavlink_system;
 
  mavlink_system.sysid = 7;                   ///< ID 20 for this airplane
  mavlink_system.type = MAV_TYPE_FLAPPING_WING;   ///< This system is an airplane / fixed wing
  mavlink_system.compid = MAV_COMP_ID_ALL;     ///< The component sending the message is the IMU, it could be also a Linux process
  
  crim::FlymapleMavlinkPacketHandler packet_handler(mavlink_system, "Serial1", 9600);
  
  while (true) {         
    mavlink_rc_channels_raw_t msg;

    msg.time_boot_ms = millis();
    msg.chan1_raw = rc.read(1);
    msg.chan2_raw = rc.read(2);
    msg.chan3_raw = rc.read(3);
    msg.chan4_raw = 0;
    msg.chan5_raw = 0;
    msg.chan6_raw = 0;
    msg.chan7_raw = 0;
    msg.chan8_raw = 0;
    msg.port = 1;
    msg.rssi = 255;
  
    packet_handler.wrap(msg);
    packet_handler.send();
    
    delay(100);
  }
   
  return 0;
}
