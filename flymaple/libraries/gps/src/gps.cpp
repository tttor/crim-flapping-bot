#include "gps/gps.h"

using namespace crim;

GlobalPositioningSystem::GlobalPositioningSystem(HardwareSerial* serial, uint32_t baud_rate, uint8_t number, uint64_t frequency) {
  setup_adafruit_gps(serial, baud_rate);
  setup_timer(number, frequency);
}

GlobalPositioningSystem::~GlobalPositioningSystem() {
  delete timer;
  delete adafruit_GPS_;
}

void GlobalPositioningSystem::setup_timer(uint8_t number, uint64_t frequency) {
  timer = new HardwareTimer(number);
  
  // Pause the timer while we're configuring it
  timer->pause();

  // Set up period
  uint64_t period;// in microseconds
  period = 1000000/frequency;
  
  timer->setPeriod(period); // in microseconds

  // Set up an interrupt on channel 1
  timer->setChannel1Mode(TIMER_OUTPUT_COMPARE);
  timer->setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update

  // Refresh the timer's count, prescale, and overflow
  timer->refresh();

  // Start the timer counting
  timer->resume();
}

void GlobalPositioningSystem::setup_adafruit_gps(HardwareSerial* serial, uint32_t baud_rate) {
  adafruit_GPS_ = new Adafruit_GPS(serial);
  
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  adafruit_GPS_->begin(baud_rate);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  adafruit_GPS_->sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data for high update rates!
  //adafruit_GPS_->sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // uncomment this line to turn on all the available data - for 9600 baud you'll want 1 Hz rate
  //adafruit_GPS_->sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);

  // Set the update rate
  // 1 Hz update rate
  //adafruit_GPS_->sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // 5 Hz update rate- for 9600 baud you'll have to set the output to RMC or RMCGGA only (see above)
  adafruit_GPS_->sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  // 10 Hz update rate - for 9600 baud you'll have to set the output to RMC only (see above)
  //adafruit_GPS_->sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

  // Request updates on antenna status, comment out to keep quiet
  adafruit_GPS_->sendCommand(PGCMD_ANTENNA);

  delay(1000);// TODO fix me
}

bool GlobalPositioningSystem::status() {
  return ( adafruit_GPS_->newNMEAreceived() and adafruit_GPS_->parse(adafruit_GPS_->lastNMEA()) );
}

void GlobalPositioningSystem::set_data(crim::GPSData* GPS_data) {
  GPS_data->set_time(adafruit_GPS_->hour, adafruit_GPS_->minute, adafruit_GPS_->seconds, adafruit_GPS_->milliseconds);
  GPS_data->set_date(adafruit_GPS_->year, adafruit_GPS_->month, adafruit_GPS_->day);
  GPS_data->set_pose(adafruit_GPS_->latitude, adafruit_GPS_->lat, adafruit_GPS_->longitude, adafruit_GPS_->lon, adafruit_GPS_->altitude);
  GPS_data->set_misc(adafruit_GPS_->speed, adafruit_GPS_->angle);
  GPS_data->set_note(adafruit_GPS_->fix, adafruit_GPS_->fixquality, adafruit_GPS_->satellites);
}

void GlobalPositioningSystem::read() {
  adafruit_GPS_->read();
}
