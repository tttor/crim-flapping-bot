#include <arduino-core/Arduino.h>
#include <comm/arduino_mavlink_packet_handler.hpp>
#include <macro/macro.h>

struct TestBenchPacket {
  mavlink_attitude_t att_msg;
  mavlink_global_position_int_t pos_msg;
  mavlink_rc_channels_raw_t rc_msg;
};

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  Serial.begin(115200);
  
  // Setup the static encoder
  const size_t encoder_out_a_pin = 2;
  const size_t encoder_out_b_pin = 3;
  const uint64_t encoder_resolution = 360;
  crim::TwoPhaseIncrementalEncoder encoder(encoder_out_a_pin, encoder_out_b_pin, encoder_resolution);
  
  // Setup the mavlink packet handler
  mavlink_system_t mavlink_system;
  mavlink_system.sysid = MAV_TYPE_TEST_BENCH;
  mavlink_system.compid = MAV_COMP_ID_STATIC_TEST_BENCH_ARDUINO;
  
  crim::ArduinoMavlinkPacketHandler packet_handler(mavlink_system, "Serial3", 9600);
  
  //
  const uint16_t rate = 10;
  uint32_t loop_counter = 0;
  while (true) {
    Serial.println("---------------------------------------------------------");
    uint8_t sysid = MAV_TYPE_TEST_BENCH;
    uint8_t compid = MAV_COMP_ID_MOBILE_TEST_BENCH_ARDUINO;
  	uint8_t msgid;
    
    //
    msgid = MAVLINK_MSG_ID_ATTITUDE;
    
    mavlink_attitude_t att_msg;
    packet_handler.wait(sysid, compid, msgid, &att_msg);
    
    Serial.print("att_msg.roll= "); Serial.println(att_msg.roll*crim::kOneRadianInDegree);
    Serial.print("att_msg.pitch= "); Serial.println(att_msg.pitch*crim::kOneRadianInDegree);
    Serial.print("att_msg.yaw= "); Serial.println(att_msg.yaw*crim::kOneRadianInDegree);
    Serial.println("");
    
    //
    msgid = MAVLINK_MSG_ID_RC_CHANNELS_RAW;
    
    mavlink_rc_channels_raw_t rc_msg;
    packet_handler.wait(sysid, compid, msgid, &rc_msg);
    Serial.print("ch_1= "); Serial.println(rc_msg.chan1_raw);
    Serial.print("ch_2= "); Serial.println(rc_msg.chan2_raw);
    Serial.print("ch_3= "); Serial.println(rc_msg.chan3_raw);
    Serial.print("ch_4= "); Serial.println(rc_msg.chan4_raw);
    Serial.println("");
    
    //
    msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
    
    mavlink_global_position_int_t enc_msg;
    packet_handler.wait(sysid, compid, msgid, &enc_msg);
    Serial.print("lat= "); Serial.println(enc_msg.lat);
    Serial.println("");
    
    ++loop_counter;
    //Serial.print("loop_counter= "); Serial.println(loop_counter);
    delay(1000/rate);
  }
  
  Serial.end();
  return 0;
}
