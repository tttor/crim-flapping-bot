#include "wirish.h"
#include "sensor/accelerometer.h"
#include "sensor/gyroscope.h"
#include "sensor/compass.h"
#include "sensor/pressure.h"

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
	init();
}   

int main(void) {
  using namespace crim;
    
  Accelerometer accelerometer;
  Gyroscope gyroscope;
  Compass compass;
  Pressure pressure;
  
  pinMode(BOARD_LED_PIN, OUTPUT);

  while(1) {
    
		Vector<double> retVal = accelerometer.getReading();
    
    SerialUSB.print("Acc ");
		SerialUSB.print(" x = "); SerialUSB.print(retVal(0)); 
    SerialUSB.print(" y = "); SerialUSB.print(retVal(1)); 
		SerialUSB.print(" z = "); SerialUSB.print(retVal(2));

		retVal = gyroscope.getReading();
    SerialUSB.print(" Gyr");
		SerialUSB.print(" x = "); SerialUSB.print(retVal(0)); 
		SerialUSB.print(" y = "); SerialUSB.print(retVal(1)); 
		SerialUSB.print(" z = "); SerialUSB.print(retVal(2));		

		retVal = compass.getReading();
    SerialUSB.print(" Mag");
		SerialUSB.print(" x = "); SerialUSB.print(retVal(0));
		SerialUSB.print(" y = "); SerialUSB.print(retVal(1));
		SerialUSB.print(" z = "); SerialUSB.print(retVal(2));

    retVal = pressure.getReading();
    SerialUSB.print(" P = "); SerialUSB.print(retVal(0));
		SerialUSB.print(" T = "); SerialUSB.print(retVal(1));
		SerialUSB.print(" Alt = "); SerialUSB.print(retVal(2));

    SerialUSB.println();

    toggleLED();
    
		delay(500);
  }
  
  return 0;
  
}
