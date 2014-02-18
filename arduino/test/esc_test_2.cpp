#include <arduino-core/Arduino.h>
#include <macro/macro.h>
#include <comm/arduino_mavlink_packet_handler.hpp>
#include <comm/four_channel_radio_control.hpp>
#include <actuator/electronics_speed_controller.hpp>

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  Serial.begin(115200);
  
  //
  const uint8_t kNChannel = 4;
  crim::rc_init(kNChannel);
  
  //
  const uint8_t pwm_pin = 8;
  crim::ElectronicsSpeedController esc(pwm_pin);
  
  const uint16_t max_throtle = 1900;
  const uint16_t min_throtle = 1100;
  esc.set_throtle_range(min_throtle, max_throtle);
  
  while (true) {
    uint16_t throtle = crim::rc_read(3);
    Serial.print("throtle= "); Serial.println(throtle);
    
    esc.set_duty_cycle(throtle);
    delay(100);
  }
  
  Serial.end();
  return 0;
}
