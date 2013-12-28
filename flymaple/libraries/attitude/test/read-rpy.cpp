#include "wirish.h"
#include "attitude/attitude.h"

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
	init();
}   

long t0,dt;

int main(void) {
  using namespace crim;
  
  Attitude attitude;
  
  pinMode(BOARD_LED_PIN, OUTPUT);

  while(1) {

    t0 = micros();
    
		attitude.read();
    
    dt = micros() - t0;

		SerialUSB.print("roll (deg) = "); SerialUSB.print(attitude.roll(DEG)); SerialUSB.print("\t");
		SerialUSB.print("pitch (deg) = "); SerialUSB.print(attitude.pitch(DEG)); SerialUSB.print("\t");
		SerialUSB.print("yaw (deg) = "); SerialUSB.print(attitude.yaw(DEG)); SerialUSB.print("\t");
    
    SerialUSB.print("roll (rad) = "); SerialUSB.print(attitude.roll()); SerialUSB.print("\t");
		SerialUSB.print("pitch (rad) = "); SerialUSB.print(attitude.pitch()); SerialUSB.print("\t");
		SerialUSB.print("yaw (rad) = "); SerialUSB.print(attitude.yaw()); SerialUSB.print("\t");
    
    SerialUSB.print("t = "); SerialUSB.print(dt); SerialUSB.print("\t");
    
  	toggleLED();

		SerialUSB.println();
    
		delay(500);
  }
  
  return 0;
  
}
