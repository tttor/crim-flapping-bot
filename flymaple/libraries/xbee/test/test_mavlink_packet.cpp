// Test communication using mavlink protocol
#include "wirish/wirish.h"
#include "mavlink/v1.0/common/mavlink.h"
#include "xbee/flymaple_mavlink_packet_handler.h"

//#define TEST_SENDING_ATTITUDE_MSG
//#define TEST_SENDING_HIGHRES_IMU_MSG
//#define TEST_SENDING_RC_MSG
//#define TEST_SENDING_RAW_IMU_MSG
#define TEST_SENDING_GPS_MSG

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
    init();
}

int main() {
  mavlink_system_t mavlink_system;
 
  mavlink_system.sysid = 7;                   ///< ID 20 for this airplane
  mavlink_system.type = MAV_TYPE_FLAPPING_WING;   ///< This system is an airplane / fixed wing
  mavlink_system.compid = MAV_COMP_ID_ALL;     ///< The component sending the message is the IMU, it could be also a Linux process
  
  // Define a dummy highres_imu msg
#ifdef TEST_SENDING_HIGHRES_IMU_MSG
  mavlink_highres_imu_t msg;

  msg.time_usec = micros();
  msg.xacc = 12.34567;
  msg.yacc = 12.34567;
  msg.zacc = 12.34567;
  msg.xgyro = 12.34567;
  msg.ygyro = 12.34567;
  msg.zgyro = 12.34567;
  msg.xmag = 12.34567;
  msg.ymag = 12.34567;
  msg.zmag = 12.34567;
  msg.abs_pressure = 12.34567;
  msg.diff_pressure = 12.34567;
  msg.pressure_alt = 12.34567;
  msg.temperature = 12.34567;
  //msg.fields_updated = 0b1111111111111;
#endif

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

#ifdef TEST_SENDING_RC_MSG
  mavlink_rc_channels_raw_t msg;

  msg.time_boot_ms = millis();
  msg.chan1_raw = 1234;
  msg.chan2_raw = 2234;
  msg.chan3_raw = 3234;
  msg.chan4_raw = 4234;
  msg.chan5_raw = 5234;
  msg.chan6_raw = 6234;
  msg.chan7_raw = 7234;
  msg.chan8_raw = 8234;
  msg.port = 1;
  msg.rssi = 255;
#endif

#ifdef TEST_SENDING_RAW_IMU_MSG
  mavlink_raw_imu_t msg;
  
  msg.time_usec = micros();
  msg.xacc = 12;
  msg.yacc = 22;
  msg.zacc = 32;
  msg.xgyro = 42;
  msg.ygyro = 52;
  msg.zgyro = 62;
  msg.xmag = 72;
  msg.ymag = 82;
  msg.zmag = 92;
#endif

#ifdef TEST_SENDING_GPS_MSG
  mavlink_gps_raw_int_t msg;
  
  msg.time_usec = micros();
  msg.lat = -1234567;
  msg.lon = -2345678;
  msg.alt = -3456789;
  msg.eph = 65535;
  msg.epv = 65535;
  msg.vel = 65535;
  msg.cog = 65535;
  msg.fix_type = 0;
  msg.satellites_visible = 0;
#endif

  //
  crim::FlymapleMavlinkPacketHandler packet_handler(mavlink_system, "Serial1", 9600);

  while (true) {
    packet_handler.wrap(msg);
    packet_handler.send();
    
    delay(500);
  }

  return 0;
}
