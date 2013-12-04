// Test packet 
#include <wirish/wirish.h>

#include <xbee/flymaple_packet.h>
#include <data-format/gps_data.h>

static const int BAUD_RATE = 9600;

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
    using namespace std;
    using namespace crim;
    
  // As if we were receiving data from a GPS sensor
  uint8_t hour, minute, seconds, year, month, day;
  uint16_t milliseconds;

  hour = 8,
  minute = 45;
  seconds = 25;
  milliseconds = 23;
  
  year = 13;
  month = 12;
  day = 4;
    
  float latitude, longitude, altitude;
  float speed, angle, magvar;
  
  latitude = 7.12345;
  longitude = 1.12345;
  altitude = 2.34455;
  
  speed = 1.23456;
  angle = 1.23456;
  magvar = 1.23456;
  
  boolean fix;
  uint8_t fixquality, satellites;
    
  fix = false;
  fixquality = 1;
  satellites = 3;
  
  // Format the data
  GPSData gps_data;
  gps_data.set_time(hour, minute, seconds, milliseconds);
  gps_data.set_date(year, month, day);
  gps_data.set_pose(latitude, longitude, altitude);
  gps_data.set_misc(speed, angle, magvar);
  gps_data.set_note(fix, fixquality, satellites);
  
  //
  FlymaplePacket packet("Serial1", BAUD_RATE);
  
  while (true) {
    packet.wrap(gps_data);
    packet.send();
    
    delay(1000);
  }
  
  return 0;
}
