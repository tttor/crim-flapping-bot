#include "wirish.h"
#include "attitude/attitude.h"
#include "mavlink/v1.0/common/mavlink.h"
#include "xbee/flymaple_mavlink_packet_handler.h"

//#define SERIAL_USB_PRINT
//#define DEBUG

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
	init();
}   

int main(void) {
  using namespace crim;
  
  mavlink_system_t mavlink_system;
 
  mavlink_system.sysid = 7;                   ///< ID 20 for this airplane
  mavlink_system.type = MAV_TYPE_FLAPPING_WING;   ///< This system is an airplane / fixed wing
  mavlink_system.compid = MAV_COMP_ID_ALL;     ///< The component sending the message is the IMU, it could be also a Linux process
  
  //
  crim::FlymapleMavlinkPacketHandler packet_handler(mavlink_system, "Serial1", 9600);
  
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
    
    mavlink_attitude_t msg;    
    
    msg.time_boot_ms = millis();
    msg.roll = roll;
    msg.pitch = pitch;
    msg.yaw = yaw;
  
    #ifdef SERIAL_USB_PRINT
    SerialUSB.print("roll= "); SerialUSB.print(roll); SerialUSB.print("\t");
    SerialUSB.print("pitch= "); SerialUSB.print(pitch); SerialUSB.print("\t");
    SerialUSB.print("yaw= "); SerialUSB.print(yaw); SerialUSB.print("\t");
    SerialUSB.println();  
    #endif
  
    packet_handler.wrap(msg);
    packet_handler.send();
    
    delay(100);
  }
  
  return 0;
  
}
