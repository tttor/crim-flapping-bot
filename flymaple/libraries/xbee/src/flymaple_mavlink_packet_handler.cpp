#include "xbee/flymaple_mavlink_packet_handler.h"

using namespace crim;
using namespace std;

FlymapleMavlinkPacketHandler::FlymapleMavlinkPacketHandler(mavlink_system_t mavlink_system, std::string port, size_t baud_rate)
    : mavlink_system_(mavlink_system), port_(port), baud_rate_(baud_rate) {
  if( port == "Serial1") {
    Serial1.begin(baud_rate_);
  } else if( port == "Serial2") {
    Serial2.begin(baud_rate_);
  } else if( port == "Serial3") {
    Serial3.begin(baud_rate_);
  }
  
  msg_ = new mavlink_message_t();
}

FlymapleMavlinkPacketHandler::~FlymapleMavlinkPacketHandler() {
  if (port_ == "SerialUSB")
    SerialUSB.end();
  else if (port_ == "Serial1") 
    Serial1.end();
  else if (port_ == "Serial2")
    Serial2.end();
  else if (port_ == "Serial3")
    Serial3.end();

  delete msg_;
}

void FlymapleMavlinkPacketHandler::send() {
  // Copy the message to the send buffer
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg_);
  
  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  if (port_ == "SerialUSB")
    SerialUSB.write(buf, len);
  else if (port_ == "Serial1") 
    Serial1.write(buf, len);
  else if (port_ == "Serial2")
    Serial2.write(buf, len);
  else if (port_ == "Serial3")
    Serial3.write(buf, len);
}

void FlymapleMavlinkPacketHandler::wrap(mavlink_attitude_t raw_msg) {
  mavlink_msg_attitude_pack(mavlink_system_.sysid, mavlink_system_.compid, msg_, 
                            raw_msg.time_boot_ms, 
                            raw_msg.roll,raw_msg.pitch,raw_msg.yaw, 
                            raw_msg.rollspeed,raw_msg.pitchspeed,raw_msg.yawspeed);
}

void FlymapleMavlinkPacketHandler::wrap(mavlink_highres_imu_t raw_msg) {
  mavlink_msg_highres_imu_pack(mavlink_system_.sysid, mavlink_system_.compid, msg_, 
                               raw_msg.time_usec, 
                               raw_msg.xacc, raw_msg.yacc, raw_msg.zacc, 
                               raw_msg.xgyro, raw_msg.ygyro, raw_msg.zgyro, 
                               raw_msg.xmag, raw_msg.ymag, raw_msg.zmag, 
                               raw_msg.abs_pressure, raw_msg.diff_pressure, raw_msg.pressure_alt, raw_msg.temperature, 
                               raw_msg.fields_updated);
}
                   
void FlymapleMavlinkPacketHandler::wrap(mavlink_rc_channels_raw_t raw_msg) {
  mavlink_msg_rc_channels_raw_pack(mavlink_system_.sysid, mavlink_system_.compid, msg_, 
                                   raw_msg.time_boot_ms,
                                   raw_msg.port,
                                   raw_msg.chan1_raw, raw_msg.chan2_raw, raw_msg.chan3_raw, raw_msg.chan4_raw, 
                                   raw_msg.chan5_raw, raw_msg.chan6_raw, raw_msg.chan7_raw, raw_msg.chan8_raw, 
                                   raw_msg.rssi);
}

void FlymapleMavlinkPacketHandler::wrap(mavlink_raw_imu_t raw_msg) {
  mavlink_msg_raw_imu_pack(mavlink_system_.sysid, mavlink_system_.compid, msg_,
                           raw_msg.time_usec,
                           raw_msg.xacc, raw_msg.yacc, raw_msg.zacc, 
                           raw_msg.xgyro, raw_msg.ygyro, raw_msg.zgyro, 
                           raw_msg.xmag, raw_msg.ymag, raw_msg.zmag);
}

void FlymapleMavlinkPacketHandler::wrap(mavlink_gps_raw_int_t raw_msg) {
  mavlink_msg_gps_raw_int_pack(mavlink_system_.sysid, mavlink_system_.compid, msg_,
                               raw_msg.time_usec, raw_msg.fix_type,
                               raw_msg.lat, raw_msg.lon, raw_msg.alt, 
                               raw_msg.eph, raw_msg.epv, raw_msg.vel, raw_msg.cog,
                               raw_msg.satellites_visible);
}
