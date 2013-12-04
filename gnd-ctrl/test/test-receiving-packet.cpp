#include <iostream>
#include <serial-port/GndCtrlPacket.h>
#include <data-format/string_data.h>

using namespace std;

int main() {
  crim::GndCtrlPacket packet("/dev/ttyUSB0", 9600, 5);
  
  size_t status;
  status = packet.receive();
  
  if (status==0) {
    crim::StringData data;
    data = packet.unwrap();
    data.print();  
  } else {
    cerr << "received, but corrupted; status= " << status << endl;
  }

  return 0;
}
