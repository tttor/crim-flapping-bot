// Unit test: RC 
#include <vector>
#include <stdint.h> // for uint16_t, uint8_t, etc
#include <string>
#include "wirish.h"
#include "rc/radio_control.h"
#include "data-format/rc_data.h"
#include "xbee/flymaple_packet_handler.h"

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
  rc_ch_pins.at(3) = D34;
  rc_ch_pins.at(4) = D35;
  rc_ch_pins.at(5) = D36;
  rc_ch_pins.at(6) = D37;
  rc_ch_pins.at(7) = D26;
  
  RadioControl rc(rc_ch_pins);
  
  // Wait for RC to be at its normal position
  // TODO make this in the RC class (?)    
  uint16_t ch_1_normal_ppm = 1508;
  uint16_t ch_2_normal_ppm = 1510;
  
  pinMode(BOARD_LED_PIN, OUTPUT);
  while( (rc.read(1) < (ch_1_normal_ppm-RadioControl::kPPMTolerance)) or 
         (rc.read(1) > (ch_1_normal_ppm+RadioControl::kPPMTolerance)) or
         (rc.read(2) < (ch_2_normal_ppm-RadioControl::kPPMTolerance)) or
         (rc.read(2) > (ch_2_normal_ppm+RadioControl::kPPMTolerance))
       ) {
    togglePin(BOARD_LED_PIN);
    delay(100);
  }
  digitalWrite(BOARD_LED_PIN, LOW);
  
  std::vector<uint16_t> init_PPMs;
  init_PPMs.resize(rc_ch_pins.size());
  for (uint8_t i=0; i<init_PPMs.size(); ++i) {
    init_PPMs.at(i) = rc.read(i+1);
  }
  
  RCData rc_data(rc_ch_pins.size());
  rc_data.set_init_PPMs(init_PPMs);
  
  FlymaplePacketHandler packet_handler("Serial3");
  
  while (true) {         
    std::vector<uint16_t> PPMs;
    PPMs.resize(rc_ch_pins.size());
    
    for (uint8_t i=0; i<PPMs.size(); ++i) {
      PPMs.at(i) = rc.read(i+1);
    }
    rc_data.set_PPMs(PPMs);

    packet_handler.wrap(rc_data);
    packet_handler.send();
    SerialUSB.println("sent");
    
    delay(100);
  }
   
  return 0;
}
