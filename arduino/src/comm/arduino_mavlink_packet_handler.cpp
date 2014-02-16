#include "arduino_mavlink_packet_handler.hpp"

using namespace crim;

ArduinoMavlinkPacketHandler::ArduinoMavlinkPacketHandler(mavlink_system_t mavlink_system, String port, uint32_t baud_rate)
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

ArduinoMavlinkPacketHandler::~ArduinoMavlinkPacketHandler() {
  if (port_ == "Serial")
    Serial.end();
  else if (port_ == "Serial1") 
    Serial1.end();
  else if (port_ == "Serial2")
    Serial2.end();
  else if (port_ == "Serial3")
    Serial3.end();

  delete msg_;
}

void ArduinoMavlinkPacketHandler::send() {
  // Copy the message to the send buffer
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg_);
  
  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  if (port_ == "Serial")
    Serial.write(buf, len);
  else if (port_ == "Serial1") 
    Serial1.write(buf, len);
  else if (port_ == "Serial2")
    Serial2.write(buf, len);
  else if (port_ == "Serial3")
    Serial3.write(buf, len);
}

void ArduinoMavlinkPacketHandler::wrap(mavlink_attitude_t raw_msg) {
  mavlink_msg_attitude_pack(mavlink_system_.sysid, mavlink_system_.compid, msg_, 
                            raw_msg.time_boot_ms, 
                            raw_msg.roll,raw_msg.pitch,raw_msg.yaw, 
                            raw_msg.rollspeed,raw_msg.pitchspeed,raw_msg.yawspeed);
}
