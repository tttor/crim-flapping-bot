#include "wirish.h"
#include "sensor/accelerometer.h"
#include "sensor/compass.h"
#include "sensor/gyroscope.h"
#include "sensor/pressure.h"


void setup() {
    // Set up the LED to blink 
    pinMode(BOARD_LED_PIN, OUTPUT);

}

void loop() {
  while(1) {
		Vector<double> retVal = Accelerometer::getReading();
    SerialUSB.print("Acc ");
		SerialUSB.print(" x = "); SerialUSB.print(retVal(0)); 
		SerialUSB.print(" y = "); SerialUSB.print(retVal(1)); 
		SerialUSB.print(" z = "); SerialUSB.print(retVal(2));

		retVal = Gyroscope::getReading();
    SerialUSB.print(" Gyr");
		SerialUSB.print(" x = "); SerialUSB.print(retVal(0)); 
		SerialUSB.print(" y = "); SerialUSB.print(retVal(1)); 
		SerialUSB.print(" z = "); SerialUSB.print(retVal(2));		

		retVal = Compass::getReading();
    SerialUSB.print(" Mag");
		SerialUSB.print(" x = "); SerialUSB.print(retVal(0));
		SerialUSB.print(" y = "); SerialUSB.print(retVal(1));
		SerialUSB.print(" z = "); SerialUSB.print(retVal(2));
    
    retVal = Pressure::getReading();
    SerialUSB.print(" P = "); SerialUSB.print(retVal(0));
		SerialUSB.print(" T = "); SerialUSB.print(retVal(1));
		SerialUSB.print(" Alt = "); SerialUSB.print(retVal(2));
    
    SerialUSB.println();

    toggleLED();
    
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
