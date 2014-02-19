#include <arduino-core/Arduino.h>
#include <comm/four_channel_radio_control.hpp>

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  Serial.begin(115200);
  
  const uint8_t kNChannel = 4;
  crim::rc_init(kNChannel);
  
  while (true) {
    for (uint8_t i=0; i<kNChannel; ++i) {
      Serial.println(crim::rc_read(i+1), DEC);
    }
    Serial.println("");
    
    delay(100);
  }
   
  Serial.end();
  return 0;
}
