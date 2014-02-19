#include <arduino-core/Arduino.h>
#include <macro/macro.h>
#include <comm/four_channel_radio_control.hpp>
#include <actuator/servo.hpp>

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  Serial.begin(115200);
  
  //
  Servo servo;
  
  const uint8_t servo_pin = 9;
  servo.attach(servo_pin);
  
  while (true) {
    uint8_t pos;
    for(pos = 0; pos <= 180; pos += 45) // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      servo.write(pos);              // tell servo to go to position in variable 'pos' 
      delay(1000);                       // waits 15ms for the servo to reach the position 
    } 
  }

  Serial.end();
  return 0;
}
