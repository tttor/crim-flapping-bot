#include "wirish.h"
#include "attitude/attitude.h"
#include "xbee/flymaple_packet_handler.h"
#include "data-format/pose_data.h"

#define SERIAL_USB_PRINT
//#define DEBUG

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
	init();
}   

int main(void) {
  using namespace crim;
  
  FlymaplePacketHandler packer_handler("Serial1", 9600);
  
  #ifndef DEBUG
  Attitude attitude;// _have_ to be outside the loop. TODO @tttor: make the costly init routine explicit
  #endif
    
  while(1) {
    #ifdef SERIAL_USB_PRINT
    SerialUSB.println("looping");
    #endif
    
    #ifndef DEBUG
    attitude.read();
    #endif
    
    double x, y, z;
    x = y = z = 0.0;

    double roll, pitch, yaw;
    roll = pitch = yaw = 0.0;
    #ifndef DEBUG
    //roll = attitude.roll(DEG);
    //pitch = attitude.pitch(DEG);
    //yaw = attitude.yaw(DEG);
    
    roll = attitude.roll();
    pitch = attitude.pitch();
    yaw = attitude.yaw();
    #endif
  
    #ifdef SERIAL_USB_PRINT
    SerialUSB.print("roll= "); SerialUSB.print(roll); SerialUSB.print("\t");
    SerialUSB.print("pitch= "); SerialUSB.print(pitch); SerialUSB.print("\t");
    SerialUSB.print("yaw= "); SerialUSB.print(yaw); SerialUSB.print("\t");
    SerialUSB.println();  
    #endif
  
    PoseData data;
    data.set_parent_frame("world");
    data.set_frame("flymaple");
    data.set_origin(x, y, z);
    data.set_rotation(roll, pitch, yaw);
    
    packer_handler.wrap(data);
    packer_handler.send();
    
    delay(150);
  }
  
  return 0;
  
}
