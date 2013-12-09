// @author vektor dewanto
#ifndef PACKET_H
#define PACKET_H

#include <string>

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// DECLARATION
namespace crim{

static const std::string PACKET_DELIMITER = "";// the trailing \n is from SerialX.println()
static const std::string DATA_FIELD_DELIMITER = ",";
static const std::string DATA_SUBFIELD_DELIMITER = ":";
static const size_t CHECKSUM_LENGTH = 32;


class Packet {
 public:
  /**
    @brief
  */
  Packet();

  /**
    @brief
  */
  ~Packet();

  /**
    @brief
  */
  void wrap_final(const std::string& data, const std::string& checksum);

 protected:
  std::string packet_;
  std::string packet_delimiter_;
  std::string data_field_delimiter_;
  std::string data_subfield_delimiter_;
  size_t checksum_length_;

 private:
   ///**
    //@brief insert field and subfiled delimiters to ease splitting
    //hh:mm:ss:mss,
  //*/
  //std::string reformat_gps_data();
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
Packet::Packet():
    packet_(std::string()),
    packet_delimiter_(PACKET_DELIMITER),
    data_field_delimiter_(DATA_FIELD_DELIMITER),
    data_subfield_delimiter_(DATA_SUBFIELD_DELIMITER),
    checksum_length_(CHECKSUM_LENGTH) {
  // nothing
}

Packet::~Packet() {
  // nothing
}

void Packet::wrap_final(const std::string& data, const std::string& checksum){
  //packet_ = data + checksum + packet_delimiter_;

  // For now (Dec 9, 2013), we rely on the built-in checksum on zigbee/xbee
  packet_ = data + packet_delimiter_;
}

}// namespace crim
#endif
