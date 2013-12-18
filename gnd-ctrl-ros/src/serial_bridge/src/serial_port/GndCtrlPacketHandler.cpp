#include <serial_port/GndCtrlPacketHandler.h>

using namespace crim;

GndCtrlPacketHandler::GndCtrlPacketHandler() {
  // do nothing
}

GndCtrlPacketHandler::GndCtrlPacketHandler(std::string port, size_t baud, size_t timeout)
    : port_(port),
      baud_(baud),
      timeout_(timeout) {
 // do nothing
}

size_t GndCtrlPacketHandler::receive() {
  using namespace std;

  size_t status = HAPPY;
  try {
    TimeoutSerial serial(port_, baud_);
    serial.setTimeout(boost::posix_time::seconds(timeout_));

    packet_ = serial.readStringUntil(PacketHandler::kPacketDelimiter);
    cerr << "packet_= " << packet_ << endl;
    
    serial.close();
    
    // check 1
    if (packet_.length()<3)
      status = NONSENSE_DATA;

    // check 2: checksum!
    // For now (Dec 9, 2013), we rely on the built-in checksum on zigbee/xbee
    //if (status == HAPPY) {
      ////
      //string r_checksum;
      //r_checksum = packet_.substr(packet_.length()-PacketHandler::kChecksumLength);

      ////
      //hashwrapper *h = new md5wrapper();
      //h->test();

      //std::string header_plus_data;
      //header_plus_data = packet_.substr(0, packet_.length()-PacketHandler::kChecksumLength);

      //std::string c_checksum;
      //c_checksum = h->getHashFromString(header_plus_data);

      //delete h;

      ////
      //if (c_checksum != r_checksum)
        //status = CHECKSUM_MISMATCH;
      //else
        //packet_ = packet_.substr(0,packet_.length()-PacketHandler::kChecksumLength); // remove checksum
    //}
  } catch(boost::system::system_error& e) {
      cerr << "Error: " << e.what() << endl;
      status = TIMEOUT;
  }

  return status;
}

crim::StringData GndCtrlPacketHandler::unwrap(){
  using namespace std;

  crim::StringData data;

  vector<string> fields;
  boost::split(fields, packet_, boost::is_any_of(PacketHandler::kDataFieldDelimiter));

  if (fields.at(0) == string()) {
    cerr << "ERR: fields.at(0) == string() \n";
    return data;
  }

  for(size_t i=0; i<fields.size(); ++i) {
    vector<string> subfields;
    boost::split(subfields, fields.at(i), boost::is_any_of(PacketHandler::kDataSubfieldDelimiter));

    data.content.push_back(subfields);
  }

  return data;
}
