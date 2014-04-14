#include <arduino-core/Arduino.h>
#include <comm/one_channel_radio_control.hpp>

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  Serial.begin(9600);
  
  crim::rc_init();
  
  while (true) {
    Serial.println(crim::rc_read(), DEC);
    delay(100);
  }
   
  Serial.end();
  return 0;
}
