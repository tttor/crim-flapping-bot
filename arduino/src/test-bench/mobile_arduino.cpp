#include <arduino-core/Arduino.h>
#include <comm/arduino_mavlink_packet_handler.hpp>
#include <comm/four_channel_radio_control.hpp>
#include <sensor/chr_um6.hpp>
#include <sensor/two_phase_incremental_encoder.hpp>

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  Serial.begin(115200);
  
  // Setup the mavlink packet handler
  mavlink_system_t mavlink_system;
  mavlink_system.sysid = MAV_TYPE_TEST_BENCH;
  mavlink_system.compid = MAV_COMP_ID_MOBILE_TEST_BENCH_ARDUINO;
  
  crim::ArduinoMavlinkPacketHandler packet_handler(mavlink_system, "Serial3", 9600);
  
  // Setup the RC
  const uint8_t kRCNChannel = 4;
  crim::rc_init(kRCNChannel);
  
  // Setup the UM6
  crim::CHR_UM6 um6;
  um6.calib();  
  
  // Setup the mobile encoder
  const size_t encoder_out_a_pin = 2;
  const size_t encoder_out_b_pin = 3;
  const uint64_t encoder_resolution = 360;
  crim::TwoPhaseIncrementalEncoder encoder(encoder_out_a_pin, encoder_out_b_pin, encoder_resolution);
  
  // Looping
  const uint16_t rate = 50;
  while (true) {
    // Send the action: RC channel values
    mavlink_rc_channels_raw_t rc_msg;
    
    rc_msg.time_boot_ms = millis();
    rc_msg.chan1_raw = crim::rc_read(1);
    rc_msg.chan2_raw = crim::rc_read(2);
    rc_msg.chan3_raw = crim::rc_read(3);
    rc_msg.chan4_raw = crim::rc_read(4);
    rc_msg.port = 1;
    rc_msg.rssi = 255;
    
    packet_handler.wrap(rc_msg);
    packet_handler.send();
    
    // Get and Send the attitude
    crim::EulerAngle euler;
    euler = um6.euler();
    
    mavlink_attitude_t att_msg;
    att_msg.roll = euler.roll;
    att_msg.pitch = euler.pitch;
    att_msg.yaw = euler.yaw;
    
    packet_handler.wrap(att_msg);
    packet_handler.send();
    
    // Send the mobile encoder data
    // We abuse the mavlink_global_position_int_t to encode encoder data
    mavlink_global_position_int_t  enc_msg;
    
    enc_msg.time_boot_ms = millis();
    enc_msg.lat = encoder.pos();
    packet_handler.wrap(enc_msg);
    packet_handler.send();
    
    delay(1000/rate);
  }
  
  Serial.end();
  return 0;
}
