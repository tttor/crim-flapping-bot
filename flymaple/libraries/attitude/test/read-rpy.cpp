#include "wirish.h"
#include "attitude/attitude.h"

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
	init();
}   

long first,last;

int main(void) {
  using namespace crim;
  
  Attitude attitude;
  
  pinMode(BOARD_LED_PIN, OUTPUT);

  while(1) {

		double roll = 0,pitch = 0,yaw = 0;

    first = micros();
		attitude.getRPY(roll,pitch,yaw);
    last = micros() - first;
		roll *= 180 / 3.1415926; pitch *= 180 / 3.1415926; yaw *= 180 / 3.1415926;

		SerialUSB.print("roll = "); SerialUSB.print(roll); SerialUSB.print("\t");
		SerialUSB.print("pitch = "); SerialUSB.print(pitch); SerialUSB.print("\t");
		SerialUSB.print("yaw = "); SerialUSB.print(yaw); SerialUSB.print("\t");
    SerialUSB.print("t = "); SerialUSB.print(last); SerialUSB.print("\t");
    
  	toggleLED();

		SerialUSB.println();
    
		delay(200);
  }
  
  return 0;
  
}
