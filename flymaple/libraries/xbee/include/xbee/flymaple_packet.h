// @author vektor dewanto
#ifndef FLYMAPLE_PACKET_H
#define FLYMAPLE_PACKET_H

#include <string>
#include <packet/packet.h>
#include <wirish.h>
#include <xbee/hl_md5wrapper_for_flymaple.h>
#include <xbee/hl_md5_for_flymaple.h>
#include <data-format/gps_data.h>

namespace crim{

static const size_t SERIAL_USB = 0;
static const size_t SERIAL_1 = 1;
static const size_t SERIAL_2 = 2;
static const size_t SERIAL_3 = 3;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DECLARATION
class FlymaplePacket: public Packet {
 public:
  /**
    @brief
  */
  FlymaplePacket(std::string port, size_t baud);

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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
FlymaplePacket::FlymaplePacket(std::string port, size_t baud): baud_(baud) {
  if (port == "SerialUSB") {
    port_ = SERIAL_USB;
  } else if( port == "Serial1") {
    port_ = SERIAL_1;
    Serial1.begin(baud_);
  } else if( port == "Serial2") {
    port_ = SERIAL_2;
    Serial2.begin(baud_);
  } else if( port == "Serial3") {
    port_ = SERIAL_3;
    Serial3.begin(baud_);
  }
}

bool FlymaplePacket::send() {
  bool status = false;
  if (port_==SERIAL_USB) {
    status = send_SerialUSB();
  } else {
    status = send_SerialX();
  }
  return status;
}

bool FlymaplePacket::send_SerialUSB() {
  SerialUSB.println(packet_.c_str());
  return true;
}

bool FlymaplePacket::send_SerialX() {
  for (size_t i=0; i<packet_.length(); ++i) {
    char c = packet_[i];

    switch (port_) {
      case SERIAL_1: {
        Serial1.write(c);
        break;
      }
      case SERIAL_2: {
        Serial2.write(c);
        break;
      }
      case SERIAL_3: {
        Serial3.write(c);
        break;
      }
      default: {
        return false;
      }
    }
  }
  return true;
}

void FlymaplePacket::wrap(const StringData& data) {
  //
  std::string f_data;// f(ormatted)_data
  size_t n_field = data.content.size();

  for(size_t i=0; i<n_field; ++i) {
    size_t n_subfield = data.content.at(i).size();
    for(size_t j=0; j<n_subfield; ++j) {
      f_data += data.content.at(i).at(j);

      if(j != n_subfield-1)
        f_data += data_subfield_delimiter_;
    }
    if(i != n_field-1)
        f_data += data_field_delimiter_;
  }

  // For now (Dec 9, 2013), we rely on the built-in checksum on zigbee/xbee
  //hashwrapper* h = new md5wrapper();
  //h->test();

  std::string checksum;
  //checksum = h->getHashFromString(f_data);

  //
  wrap_final(f_data, checksum);
  //delete h;
}

}// namespace crim
#endif
