// @author vektor dewanto
#ifndef FLYMAPLE_MAVLINK_PACKET_HANDLER_H
#define FLYMAPLE_MAVLINK_PACKET_HANDLER_H

#include <string>
#include "wirish.h"
#include "mavlink/v1.0/common/mavlink.h"

namespace crim {

class FlymapleMavlinkPacketHandler {
 public:
  /**
   * @brief
   */
  FlymapleMavlinkPacketHandler(mavlink_system_t mavlink_system, std::string port, size_t baud_rate=9600);

  /**
   * @brief
   */
  ~FlymapleMavlinkPacketHandler();

  /**
   * @brief
   */
  void send();
    
  /**
   * @brief
   */
  void wrap(mavlink_attitude_t raw_msg);
  
  /**
   * @brief
   */
  void wrap(mavlink_highres_imu_t raw_msg);

  /**
   * @brief
   */
  void wrap(mavlink_rc_channels_raw_t raw_msg);
  
  /**
   * @brief
   */
  void wrap(mavlink_raw_imu_t raw_msg);
  
 private:
  mavlink_system_t mavlink_system_;
  std::string port_;
  size_t baud_rate_;
  mavlink_message_t* msg_;

  //bool send_SerialUSB();
  //bool send_SerialX();
};

}// namespace crim
#endif
