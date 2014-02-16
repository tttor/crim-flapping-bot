// @author vektor dewanto
#ifndef ARDUINO_MAVLINK_PACKET_HANDLER
#define ARDUINO_MAVLINK_PACKET_HANDLER

#include <arduino-core/Arduino.h>
#include <mavlink/v1.0/common/mavlink.h>

namespace crim {

class ArduinoMavlinkPacketHandler {
 public:
   /**
   * @brief
   */
  ArduinoMavlinkPacketHandler(mavlink_system_t mavlink_system, String port, uint32_t baud_rate=9600);

  /**
   * @brief
   */
  ~ArduinoMavlinkPacketHandler();

  /**
   * @brief
   */
  void send();
    
  /**
   * @brief
   */
  void wrap(mavlink_attitude_t raw_msg);
 
 private:
  mavlink_system_t mavlink_system_;
  String port_;
  uint32_t baud_rate_;
  mavlink_message_t* msg_;
};

}// namespace crim

#endif
