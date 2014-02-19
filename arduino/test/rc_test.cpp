#include <arduino-core/Arduino.h>
#include <comm/radio_control.hpp>

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  Serial.begin(115200);
  
  const uint8_t kNChannel = 1;
  crim::RadioControl rc(kNChannel);
  
  while (true) {
    for (uint8_t i=0; i<kNChannel; ++i) {
      Serial.println(rc.read(i+1), DEC);
    }
    Serial.println("");
    
    delay(200);
  }
   
  Serial.end();
  return 0;
}
