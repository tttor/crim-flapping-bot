#include "wirish.h"
#include "attitude/attitude.h"
#include "xbee/flymaple_packet_handler.h"
#include "data-format/pose_data.h"

//#define DEBUG

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
	init();
}   

int main(void) {
  using namespace crim;
  
  FlymaplePacketHandler packer_handler("Serial3", 9600);
  
  while(1) {
    SerialUSB.println("looping");
    
    #ifndef DEBUG
    Attitude attitude;
    attitude.read();
    #endif
    
    double x, y, z;
    x = y = z = 0.0;

    double roll, pitch, yaw;
    roll = pitch = yaw = 0.0;
    #ifndef DEBUG
    roll = attitude.roll();
    pitch = attitude.pitch();
    yaw = attitude.yaw();
    #endif
  
    SerialUSB.print("roll (rad) = "); SerialUSB.print(roll); SerialUSB.print("\t");
    SerialUSB.print("pitch (rad) = "); SerialUSB.print(pitch); SerialUSB.print("\t");
    SerialUSB.print("yaw (rad) = "); SerialUSB.print(yaw); SerialUSB.print("\t");
    SerialUSB.println();  
  
    PoseData data;
    data.set_parent_frame("world");
    data.set_frame("flymaple");
    data.set_origin(x, y, z);
    data.set_rotation(roll, pitch, yaw);
    
    packer_handler.wrap(data);
    packer_handler.send();
    
    delay(100);
  }
  
  return 0;
  
}
