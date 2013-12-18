// @author vektor dewanto
#ifndef FLYMAPLE_PACKET_HANDLER_H
#define FLYMAPLE_PACKET_HANDLER_H

#include "wirish.h"

#include "packet/packet_handler.h"
#include "data-format/string_data.h"
//#include "xbee/hl_md5wrapper_for_flymaple.h"// TODO Fix this header to include
//#include "xbee/hl_md5_for_flymaple.h"// TODO Fix this header to include

namespace crim {

static const size_t SERIAL_USB = 0;
static const size_t SERIAL_1 = 1;
static const size_t SERIAL_2 = 2;
static const size_t SERIAL_3 = 3;

class FlymaplePacketHandler: public PacketHandler {
 public:
  /**
    @brief
  */
  FlymaplePacketHandler(std::string port, size_t baud=9600);

  /**
    @brief
  */
  bool send();

  /**
    @brief
  */
  void wrap(const StringData& data);

 private:
  size_t port_;
  size_t baud_;

  bool send_SerialUSB();
  bool send_SerialX();
};

}// namespace crim
#endif
