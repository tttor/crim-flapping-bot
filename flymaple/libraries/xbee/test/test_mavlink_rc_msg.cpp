// Test communication using mavlink protocol
#include "wirish/wirish.h"
#include "mavlink/v1.0/common/mavlink.h"

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
    init();
}

int main() {
  mavlink_system_t mavlink_system;
 
  mavlink_system.sysid = 7;                   ///< ID 20 for this airplane
  mavlink_system.type = MAV_TYPE_FLAPPING_WING;   ///< This system is an airplane / fixed wing
  mavlink_system.compid = MAV_COMP_ID_ALL;     ///< The component sending the message is the IMU, it could be also a Linux process
  
  // Define the msg
  mavlink_rc_channels_raw_t raw_msg;
  raw_msg.time_boot_ms = millis();
  raw_msg.chan1_raw = 1234;
  raw_msg.chan2_raw = 2234;
  raw_msg.chan3_raw = 3234;
  raw_msg.chan4_raw = 4234;
  raw_msg.chan5_raw = 5234;
  raw_msg.chan6_raw = 6234;
  raw_msg.chan7_raw = 7234;
  raw_msg.chan8_raw = 8234;
  raw_msg.port = 1;
  raw_msg.rssi = 255;
  
  //
  Serial1.begin(9600);

  while (true) {
    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    // Pack the message
    mavlink_msg_rc_channels_raw_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 
                                     raw_msg.time_boot_ms,
                                     raw_msg.port,
                                     raw_msg.chan1_raw, raw_msg.chan2_raw, raw_msg.chan3_raw, raw_msg.chan4_raw, 
                                     raw_msg.chan5_raw, raw_msg.chan6_raw, raw_msg.chan7_raw, raw_msg.chan8_raw, 
                                     raw_msg.rssi);
    
    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    
    // Send the message with the standard UART send function
    // uart0_send might be named differently depending on
    // the individual microcontroller / library in use.
    Serial1.write(buf, len);
    
    delay(500);
  }

  return 0;
}
