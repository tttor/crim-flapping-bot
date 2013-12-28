#include "wirish.h"
#include "sensor/accelerometer.h"
#include "sensor/gyroscope.h"
#include "sensor/compass.h"
#include "sensor/barometer.h"

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
	init();
}   

int main(void) {
  using namespace crim;
  using namespace std;
    
  Accelerometer accelerometer;
  Gyroscope gyroscope;
  Compass compass;
  Barometer pressure;
  
  pinMode(BOARD_LED_PIN, OUTPUT);

  while(1) {
    
		//vector<double> retVal = accelerometer.getReading();
    accelerometer.read();
    
    SerialUSB.print("Acc ");
		SerialUSB.print(" x = "); SerialUSB.print(accelerometer.x()); 
    SerialUSB.print(" y = "); SerialUSB.print(accelerometer.y()); 
		SerialUSB.print(" z = "); SerialUSB.print(accelerometer.z());

		gyroscope.read();
    SerialUSB.print(" Gyr");
	  SerialUSB.print(" x = "); SerialUSB.print(gyroscope.x()); 
    SerialUSB.print(" y = "); SerialUSB.print(gyroscope.y()); 
		SerialUSB.print(" z = "); SerialUSB.print(gyroscope.z());

		compass.read();
    SerialUSB.print(" Mag");
    SerialUSB.print(" x = "); SerialUSB.print(compass.x()); 
    SerialUSB.print(" y = "); SerialUSB.print(compass.y()); 
		SerialUSB.print(" z = "); SerialUSB.print(compass.z());

    pressure.read();
    SerialUSB.print(" Press");
    SerialUSB.print(" P = "); SerialUSB.print(pressure.getPressure()); 
    SerialUSB.print(" temp = "); SerialUSB.print(pressure.getTemperature()); 
		SerialUSB.print(" Alt = "); SerialUSB.print(pressure.getAltitude());
   

    SerialUSB.println();
    toggleLED();
    
		delay(500);
  }
  
  return 0;
  
}
