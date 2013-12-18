#include "packet/packet_handler.h"

using namespace crim;

const std::string PacketHandler::kPacketDelimiter = "\n";
const std::string PacketHandler::kDataFieldDelimiter = ",";
const std::string PacketHandler::kDataSubfieldDelimiter = ":";
const size_t PacketHandler::kChecksumLength = 32;

PacketHandler::PacketHandler()
{
  packet_ = std::string();
}

PacketHandler::~PacketHandler() {
  // nothing
}

void PacketHandler::wrap_final(const std::string& data, const std::string& checksum){
  //packet_ = data + checksum + PacketHandler::kPacketDelimiter;

  // For now (Dec 9, 2013), we rely on the built-in checksum on zigbee/xbee
  packet_ = data + PacketHandler::kPacketDelimiter;
}
