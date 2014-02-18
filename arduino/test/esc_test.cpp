#include <arduino-core/Arduino.h>
#include <macro/macro.h>
#include <comm/arduino_mavlink_packet_handler.hpp>
#include <actuator/electronics_speed_controller.hpp>

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  Serial.begin(115200);
  
  const uint8_t pwm_pin = 8;
  crim::ElectronicsSpeedController esc(pwm_pin);
  
  while (true) {
    for (uint8_t i=0; i<100; ++i) {
      double duty_cycle = i;
      Serial.print("duty_cycle= "); Serial.println(duty_cycle);
      
      esc.set_duty_cycle(duty_cycle);
      delay(500);
    }
  }
  
  Serial.end();
  return 0;
}
