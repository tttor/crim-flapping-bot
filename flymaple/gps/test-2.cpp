// Unit test: GPS
#include <stdlib.h> // for Adafruit_GPS lib, and this _must_ be on top of all #include
#include <wirish/wirish.h>

#include <gps/Adafruit_GPS.h>

Adafruit_GPS GPS(&Serial1);

void setup() {
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
    setup();

    //// Print raw data
    //Serial2.begin(9600);
    //while (true) {
      //char c = GPS.read();

      //Serial2.print(c);
      //Serial2.println();

      //delay(1000);
    //}

    // Print parsed data
    Serial2.begin(9600);
    while (true) {
       // if a sentence is received, we can check the checksum, parse it...
      if (GPS.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences!
        // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
        //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

        if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
          continue;  // we can fail to parse a sentence in which case we should just wait for another
      }

      Serial2.print("\nTime: ");
      Serial2.print(GPS.hour, DEC); Serial2.print(':');
      Serial2.print(GPS.minute, DEC); Serial2.print(':');
      Serial2.print(GPS.seconds, DEC); Serial2.print('.');
      Serial2.println(GPS.milliseconds);

      Serial2.print("Date: ");
      Serial2.print(GPS.day, DEC); Serial2.print('/');
      Serial2.print(GPS.month, DEC); Serial2.print('/');
      Serial2.print("20"); Serial2.println(GPS.year, DEC);

      Serial2.print("Fix: "); Serial2.print((int)GPS.fix);
      Serial2.print(" quality: "); Serial2.println((int)GPS.fixquality);
      if (GPS.fix) {
        Serial2.print("Location: ");
        Serial2.print(GPS.latitude, 4); Serial2.print(GPS.lat);
        Serial2.print(", ");
        Serial2.print(GPS.longitude, 4); Serial2.println(GPS.lon);

        Serial2.print("Speed (knots): "); Serial2.println(GPS.speed);
        Serial2.print("Angle: "); Serial2.println(GPS.angle);
        Serial2.print("Altitude: "); Serial2.println(GPS.altitude);
        Serial2.print("Satellites: "); Serial2.println((int)GPS.satellites);
      }

      delay(1000);
    }

    return 0;
}
