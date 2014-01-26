// Test communication using mavlink protocol
#include "wirish/wirish.h"
#include "mavlink/v1.0/common/mavlink.h"
#include "xbee/flymaple_mavlink_packet_handler.h"

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
    init();
}

int main() {
  mavlink_system_t mavlink_system;
 
  mavlink_system.sysid = 7;                   ///< ID 20 for this airplane
  mavlink_system.type = MAV_TYPE_FLAPPING_WING;   ///< This system is an airplane / fixed wing
  mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  
  // Define the attitude msg
  mavlink_attitude_t attitude_msg;
  attitude_msg.roll = 0.;
  attitude_msg.pitch = 0.;
  attitude_msg.yaw = 0.;
  attitude_msg.rollspeed = 67.890;
  attitude_msg.pitchspeed = 43.542;
  attitude_msg.yawspeed = 27.344;
  
  //
  crim::FlymapleMavlinkPacketHandler packet_handler(mavlink_system, "Serial1", 9600);

  while (true) {
    packet_handler.wrap(MAVLINK_MSG_ID_ATTITUDE, attitude_msg);
    packet_handler.send();
    
    delay(500);
  }

  return 0;
}
