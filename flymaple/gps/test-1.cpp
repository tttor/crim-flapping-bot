// Unit test: GPS
#include <stdlib.h> // for Adafruit_GPS lib, and this _must_ be on top of all #include
#include <wirish/wirish.h>

#include <gps/Adafruit_GPS.cpp>

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
    
    // Print raw data
    //while (true) {
      //char c = GPS.read();
      
      //SerialUSB.print(c);
      ////SerialUSB.println();
    //}
    
    // Print parsed data
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

      SerialUSB.print("\nTime: ");
      SerialUSB.print(GPS.hour, DEC); SerialUSB.print(':');
      SerialUSB.print(GPS.minute, DEC); SerialUSB.print(':');
      SerialUSB.print(GPS.seconds, DEC); SerialUSB.print('.');
      SerialUSB.println(GPS.milliseconds);
      
      SerialUSB.print("Date: ");
      SerialUSB.print(GPS.day, DEC); SerialUSB.print('/');
      SerialUSB.print(GPS.month, DEC); SerialUSB.print('/');
      SerialUSB.print("20"); SerialUSB.println(GPS.year, DEC);
      
      SerialUSB.print("Fix: "); SerialUSB.print((int)GPS.fix);
      SerialUSB.print(" quality: "); SerialUSB.println((int)GPS.fixquality); 
      if (GPS.fix) {
        SerialUSB.print("Location: ");
        SerialUSB.print(GPS.latitude, 4); SerialUSB.print(GPS.lat);
        SerialUSB.print(", "); 
        SerialUSB.print(GPS.longitude, 4); SerialUSB.println(GPS.lon);
        
        SerialUSB.print("Speed (knots): "); SerialUSB.println(GPS.speed);
        SerialUSB.print("Angle: "); SerialUSB.println(GPS.angle);
        SerialUSB.print("Altitude: "); SerialUSB.println(GPS.altitude);
        SerialUSB.print("Satellites: "); SerialUSB.println((int)GPS.satellites);
      }
      
      delay(2000);
    }
    
    return 0;
}
