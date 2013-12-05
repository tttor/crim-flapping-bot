// @author vektor dewanto
#ifndef GPS_DATA_H
#define	GPS_DATA_H

#include <string>
#include <stdio.h> // for snprintf()

#include <data-format/string_data.h>

using namespace std;

namespace crim{

static const int BUFFER_CAPACITY = 100;
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// DECLARATION
struct GPSData: public StringData {
  GPSData();
  ~GPSData();

  void set_time(uint8_t hour, uint8_t minute, uint8_t second, uint16_t millisecond);
  void set_date(uint8_t year, uint8_t month, uint8_t day);
  void set_pose(float latitude, float longitude, float altitude);
  void set_misc(float angle, float speed, float magvar);
  void set_note(boolean fix, uint8_t fixquality, uint8_t n_satellite);
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
GPSData::GPSData() {
  content.resize(6);// 0->id, 1->time, 2->date, 3->pose, 4->misc, 5-note
  content.at(0).push_back("GPS");// data id
}

GPSData::~GPSData() {
  // nothing
}

void GPSData::set_note(boolean fix, uint8_t fixquality, uint8_t n_satellite) {
  vector<uint8_t> argv;
  argv.push_back((uint8_t)fix);
  argv.push_back(fixquality);
  argv.push_back(n_satellite);
  
  //
  vector<string> field;
  field.resize(3);
    
  for(size_t i=0; i<3; ++i) {
    char tmp[BUFFER_CAPACITY];
    
    int status;
    status = snprintf(tmp, BUFFER_CAPACITY, "%d", argv.at(i));
    
    if ((status>0) && (status<=BUFFER_CAPACITY)) 
      field.at(i) = tmp;
  }
  
  //
  content.at(5) = field;
}

void GPSData::set_misc(float angle, float speed, float magvar) {
  vector<float> argv;
  argv.push_back(angle);
  argv.push_back(speed);
  argv.push_back(magvar);
  
  //
  vector<string> field;
  field.resize(3);
  
  for(size_t i=0; i<3; ++i) {
    char tmp[BUFFER_CAPACITY];
    
    int status;
    status = snprintf(tmp, BUFFER_CAPACITY, "%.5f", argv.at(i));
    
    if ((status>0) && (status<=BUFFER_CAPACITY)) 
      field.at(i) = tmp;
  }
  
  //
  content.at(4) = field;
}

void GPSData::set_pose(float latitude, float longitude, float altitude) {
  vector<float> argv;
  argv.push_back(latitude);
  argv.push_back(longitude);
  argv.push_back(altitude);
  
  //
  vector<string> field;
  field.resize(3);
  
  for(size_t i=0; i<3; ++i) {
    char tmp[BUFFER_CAPACITY];
    
    int status;
    status = snprintf(tmp, BUFFER_CAPACITY, "%.5f", argv.at(i));
    
    if ((status>0) && (status<=BUFFER_CAPACITY)) 
      field.at(i) = tmp;
  }
  
  //
  content.at(3) = field;
}

void GPSData::set_date(uint8_t year, uint8_t month, uint8_t day) {
  vector<uint8_t> argv;
  argv.push_back(year);
  argv.push_back(month);
  argv.push_back(day);
  
  //
  vector<string> field;
  field.resize(3);
  
  for(size_t i=0; i<3; ++i) {
    char tmp[BUFFER_CAPACITY];
    
    int status;
    status = snprintf(tmp, BUFFER_CAPACITY, "%d", argv.at(i));
    
    if ((status>0) && (status<=BUFFER_CAPACITY)) 
      field.at(i) = tmp;
  }
  
  //
  content.at(2) = field;
}

void GPSData::set_time(uint8_t hour, uint8_t minute, uint8_t second, uint16_t millisecond) {
  vector<uint16_t> argv;
  argv.push_back(hour);
  argv.push_back(minute);
  argv.push_back(second);
  argv.push_back(millisecond);
  
  //
  vector<string> field;
  field.resize(4);
  
  for(size_t i=0; i<4; ++i) {
    char tmp[BUFFER_CAPACITY];
    
    int status;
    status = snprintf(tmp, BUFFER_CAPACITY, "%d", argv.at(i));
    
    if ((status>0) && (status<=BUFFER_CAPACITY)) 
      field.at(i) = tmp;
  }
  
  //
  content.at(1) = field;
};

}// namespace crim
#endif
