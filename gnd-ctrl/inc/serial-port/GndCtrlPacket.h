// @author vektor dewanto
#ifndef GND_CTRL_PACKET_H
#define	GND_CTRL_PACKET_H

#include <string>
#include <packet/packet.h>

#include <serial-port/TimeoutSerial.h>
#include <serial-port/hl_hashwrapper.h>
#include <serial-port/hl_md5wrapper.h>

#include <data-format/string_data.h>

#include <boost/algorithm/string.hpp>

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// DECLARATION
namespace crim{

const size_t HAPPY = 0;
const size_t CHECKSUM_MISMATCH = 1;
const size_t NONSENSE_DATA = 2;// either empty or even header-only is not complete
const size_t TIMEOUT = 3;
  
class GndCtrlPacket: public Packet {
 public:
  /**
    @brief
  */
  GndCtrlPacket(std::string port, size_t baud, size_t timeout);

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
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
GndCtrlPacket::GndCtrlPacket(std::string port, size_t baud, size_t timeout)
    : port_(port), 
      baud_(baud),
      timeout_(timeout) {

}

size_t GndCtrlPacket::receive() {
  using namespace std;
  
  size_t status = HAPPY;
  try {
    TimeoutSerial serial(port_, baud_);
    serial.setTimeout(boost::posix_time::seconds(timeout_));

    packet_ = serial.readStringUntil(packet_delimiter_);
    serial.close();
    
    // check 1
    if (packet_.length()<3)
      status = NONSENSE_DATA;
    
    // check 2: checksum!
    if (status == HAPPY) {
      //
      string r_checksum;
      r_checksum = packet_.substr(packet_.length()-checksum_length_);
      
      //
     	hashwrapper *h = new md5wrapper();
      h->test();
  
      std::string header_plus_data;
      header_plus_data = packet_.substr(0, packet_.length()-checksum_length_);
      
      std::string c_checksum;
      c_checksum = h->getHashFromString(header_plus_data);
	
      delete h;
      
      //
      if (c_checksum != r_checksum)
        status = CHECKSUM_MISMATCH;
      else
        packet_ = packet_.substr(0,packet_.length()-checksum_length_); // remove checksum
    }
  } catch(boost::system::system_error& e) {
      cerr << "Error: " << e.what() << endl;
      status = TIMEOUT;
  }
  
  return status;
}

crim::StringData 
GndCtrlPacket::unwrap(){
  using namespace std;
  
  crim::StringData data;
  
  vector<string> fields;
  boost::split(fields, packet_, boost::is_any_of(data_field_delimiter_));
  
  if (fields.at(0) == string()) {
    cerr << "ERR: fields.at(0) == string() \n";
    return data;
  } 
  
  for(size_t i=0; i<fields.size(); ++i) {
    vector<string> subfields;
    boost::split(subfields, fields.at(i), boost::is_any_of(data_subfield_delimiter_));
    
    data.content.push_back(subfields);
  }
  
  return data;
}

}// namespace crim
#endif
