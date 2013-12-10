#include "packet/packet.h"

using namespace crim;

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
