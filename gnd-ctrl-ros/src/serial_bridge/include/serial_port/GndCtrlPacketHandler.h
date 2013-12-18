// @author vektor dewanto
#ifndef GND_CTRL_PACKET_HANDLER_H
#define GND_CTRL_PACKET_HANDLER_H

#include <string>
#include <packet/packet_handler.h>

#include <serial_port/TimeoutSerial.h>
#include <serial_port/hl_hashwrapper.h>
#include <serial_port/hl_md5wrapper.h>

#include <data-format/string_data.h>

#include <boost/algorithm/string.hpp>

namespace crim{

const size_t HAPPY = 0;
const size_t CHECKSUM_MISMATCH = 1;
const size_t NONSENSE_DATA = 2;// either empty or even header-only is not complete
const size_t TIMEOUT = 3;

class GndCtrlPacketHandler: public PacketHandler {
 public:
   /**
    @brief
  */
  GndCtrlPacketHandler();

  /**
    @brief
  */
  GndCtrlPacketHandler(std::string port, size_t baud, size_t timeout);

  /**
    @brief
  */
  size_t receive();

  /**
    @brief
  */
  crim::StringData unwrap();

 private:
  std::string port_;
  size_t baud_;
  size_t timeout_;
};

}// namespace crim
#endif
