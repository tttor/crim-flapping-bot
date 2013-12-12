#include "wirish.h"
#include "attitude/attitude.h"

long first,last;

void setup() {
    // Set up the LED to blink 
    pinMode(BOARD_LED_PIN, OUTPUT);

}

void loop() {
  while(1) {

		double roll = 0,pitch = 0,yaw = 0;

    first=micros();
    
		Attitude::getRPY(roll,pitch,yaw);

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
    
	SerialUSB.println();
}

/* Please Do Not Remove & Edit Following Code */
int main(void) {
    setup();

    while (true) {
        loop();
    }

    return 0;
}
