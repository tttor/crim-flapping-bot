// @author vektor dewanto
#ifndef PACKET_HANDLER_H
#define PACKET_HANDLER_H

#include <string>

namespace crim{

class PacketHandler {
 public:
  /**
    @brief
  */
  PacketHandler();

  /**
    @brief
  */
  ~PacketHandler();

  /**
    @brief
  */
  void wrap_final(const std::string& data, const std::string& checksum);
  
  static const std::string kPacketDelimiter;
  static const std::string kDataFieldDelimiter;
  static const std::string kDataSubfieldDelimiter;
  static const size_t kChecksumLength;
   
 protected:
  std::string packet_;

 private:
 
};

}// namespace crim
#endif
