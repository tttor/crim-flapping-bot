// Unit test: GPS via Xbee
#include <stdlib.h> // for Adafruit_GPS lib, and this _must_ be on top of all #include
#include <wirish/wirish.h>

#include <gps/Adafruit_GPS.h>

#include <xbee/flymaple_packet.h>
#include <data-format/gps_data.h>

static const int BAUD_RATE = 9600;

Adafruit_GPS GPS(&Serial1);

void setup_gps() {
  // 9600 NMEA is the default baud rate for MTK - some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data for high update rates!
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // uncomment this line to turn on all the available data - for 9600 baud you'll want 1 Hz rate
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  
  // Set the update rate
  // 1 Hz update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // 5 Hz update rate- for 9600 baud you'll have to set the output to RMC or RMCGGA only (see above)
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  // 10 Hz update rate - for 9600 baud you'll have to set the output to RMC only (see above)
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
}

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
  setup_gps();

  while (true) {
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        continue;  // we can fail to parse a sentence in which case we should just wait for another
    }
      
    // Format the data
    crim::GPSData gps_data;
    gps_data.set_time(GPS.hour, GPS.minute, GPS.seconds, GPS.milliseconds);
    gps_data.set_date(GPS.year, GPS.month, GPS.day);
    gps_data.set_pose(GPS.latitude, GPS.lat, GPS.longitude, GPS.lon, GPS.altitude);
    gps_data.set_misc(GPS.speed, GPS.angle);
    gps_data.set_note(GPS.fix, GPS.fixquality, GPS.satellites);
    
    crim::FlymaplePacket packet("Serial2", BAUD_RATE);
    packet.wrap(gps_data);
    packet.send();
    
    delay(1000);
  }
  return 0;
}
