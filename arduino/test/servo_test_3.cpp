#include <arduino-core/Arduino.h>
#include <macro/macro.h>
#include <comm/four_channel_radio_control.hpp>
#include <actuator/servo.hpp>

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  Serial.begin(115200);
  
  //
  const uint8_t kNChannel = 4;
  crim::rc_init(kNChannel);
  
  //
  const uint8_t servo_pin = 9;
  const uint8_t servo2_pin = 10;
  
  const uint8_t servo_ch = 1;
  const uint8_t servo2_ch = 2;

  Servo servo;
  servo.attach(servo_pin);
  servo.setSpeed(352.941176471);// degree per second, = 0.17s/60degree
  servo.setThrotleRange(1100, 1920);
  servo.setNeutralThrotlePos(1520, 90);
  
  Servo servo2;
  servo2.attach(servo2_pin);
  servo2.setSpeed(352.941176471);// degree per second, = 0.17s/60degree
  servo2.setThrotleRange(1100, 1920);
  servo2.setNeutralThrotlePos(1520, 90);
  
  while (true) {
    uint16_t throtle = crim::rc_read(servo_ch);
    Serial.print("throtle= "); Serial.println(throtle);
    
    uint16_t throtle2 = crim::rc_read(servo2_ch);
    Serial.print("throtle2= "); Serial.println(throtle2);
    
    servo.writeThrotle(throtle);
    servo2.writeThrotle(throtle2);
  }

  Serial.end();
  return 0;
}
