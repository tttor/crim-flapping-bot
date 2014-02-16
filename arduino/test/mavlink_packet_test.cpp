#include <arduino-core/Arduino.h>
#include <comm/arduino_mavlink_packet_handler.hpp>

#define TEST_SENDING_ATTITUDE_MSG

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  Serial.begin(115200);
  
  mavlink_system_t mavlink_system;
 
  mavlink_system.sysid = 7;                   ///< ID 20 for this airplane
  mavlink_system.type = MAV_TYPE_FLAPPING_WING;   ///< This system is an airplane / fixed wing
  mavlink_system.compid = MAV_COMP_ID_ALL;     ///< The component sending the message is the IMU, it could be also a Linux process
  
  #ifdef TEST_SENDING_ATTITUDE_MSG  
   // Define the attitude msg
  mavlink_attitude_t msg;

  msg.time_boot_ms = millis();
  msg.roll = 0.;
  msg.pitch = 0.;
  msg.yaw = 0.;
  msg.rollspeed = 67.890;
  msg.pitchspeed = 43.542;
  msg.yawspeed = 27.344;
#endif

  //
  crim::ArduinoMavlinkPacketHandler packet_handler(mavlink_system, "Serial3", 9600);

  while (true) {
    packet_handler.wrap(msg);
    packet_handler.send();
    
    delay(500);
  }
  
  Serial.end();
  return 0;
}
