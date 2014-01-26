// Test communication using mavlink protocol
#include "wirish/wirish.h"
#include "mavlink/v1.0/common/mavlink.h"

//#define TEST_SENDING_ATTITUDE_MSG
//#define TEST_SENDING_HIGHRES_IMU_MSG
#define TEST_SENDING_RC_MSG

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
    init();
}

int main() {
  mavlink_system_t mavlink_system;
 
  mavlink_system.sysid = 7;                   ///< ID 20 for this airplane
  mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  mavlink_system.type = MAV_TYPE_FLAPPING_WING;   ///< This system is an airplane / fixed wing
  
  // Define the attitude msg
  mavlink_attitude_t mavlink_attitude;
  mavlink_attitude.roll = 12.345;
  mavlink_attitude.pitch = 12.345;
  mavlink_attitude.yaw = 12.345;
  mavlink_attitude.rollspeed = 12.345;
  mavlink_attitude.pitchspeed = 12.345;
  mavlink_attitude.yawspeed = 12.345;
  uint32_t time_boot_ms = millis();

  while (true) {
    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    // Pack the message
    mavlink_msg_attitude_pack(mavlink_system.sysid, mavlink_system.compid, &msg, time_boot_ms, mavlink_attitude.roll, mavlink_attitude.pitch, mavlink_attitude.yaw, mavlink_attitude.rollspeed, mavlink_attitude.pitchspeed, mavlink_attitude.yawspeed);
    
    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    
    // Send the message with the standard UART send function
    // uart0_send might be named differently depending on
    // the individual microcontroller / library in use.
    SerialUSB.write(buf, len);
    
    delay(500);
  }

  return 0;
}
