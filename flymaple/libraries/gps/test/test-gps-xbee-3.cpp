#include <stdlib.h> // for Adafruit_GPS lib, and this _must_ be on top of all #include
#include "wirish.h"
#include "gps/gps.h"
#include "xbee/flymaple_packet_handler.h"
#include "data-format/gps_data.h"

crim::GlobalPositioningSystem* g_GPS;

void timer_int_handler(void) {
  g_GPS->read();
}

void setup_timer_int_for_GPS() {
  g_GPS->timer->attachCompare1Interrupt(timer_int_handler);
}

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {
    init();
}

int main() {
  g_GPS = new crim::GlobalPositioningSystem(&Serial2, 9600, 2, 1000);
  setup_timer_int_for_GPS();
  
  while(true) {
    SerialUSB.println("looping");
    
    if (g_GPS->status()) {
      crim::GPSData gps_data;
      g_GPS->set_data(&gps_data);

      crim::FlymaplePacketHandler packet_handler("Serial1",9600);
      packet_handler.wrap(gps_data);
      packet_handler.send();
    } else {
      SerialUSB.println("g_GPS->status() == false");
    }
  
    delay(100);
  }
  
  return 0;
}
