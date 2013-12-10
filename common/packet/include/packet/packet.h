// @author vektor dewanto
#ifndef PACKET_H
#define PACKET_H

#include <string>

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
 
};

}// namespace crim
#endif
