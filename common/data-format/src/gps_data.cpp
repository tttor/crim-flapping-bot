#include "data-format/gps_data.h"

using namespace crim;
using namespace std;

GPSData::GPSData() {
  content.resize(6);// 0->id, 1->time, 2->date, 3->pose, 4->misc, 5-note
  content.at(0).push_back("GPS");// data id
}

GPSData::~GPSData() {
  // nothing
}

void GPSData::set_note(bool fix, uint8_t fixquality, uint8_t n_satellite) {
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

void GPSData::set_misc(float angle, float speed) {
  vector<float> argv;
  argv.push_back(angle);
  argv.push_back(speed);
  
  //
  vector<string> field;
  field.resize(2);
  
  for(size_t i=0; i<2; ++i) {
    char tmp[BUFFER_CAPACITY];
    
    int status;
    status = snprintf(tmp, BUFFER_CAPACITY, "%.5f", argv.at(i));
    
    if ((status>0) && (status<=BUFFER_CAPACITY)) 
      field.at(i) = tmp;
  }
  
  //
  content.at(4) = field;
}

void GPSData::set_pose(float latitude, char lat, float longitude, char lon, float altitude) {
  vector<float> argv;
  argv.push_back(latitude);
  argv.push_back(longitude);
  argv.push_back(altitude);
  
  vector<char> argv_2;
  argv_2.push_back(lat);
  argv_2.push_back(lon);
  
  vector<string> field;
  field.resize(5);
  
  for(size_t i=0; i<5; ++i) {
    char tmp[BUFFER_CAPACITY];
    
    int status;
    switch (i) {
      case 0: {
        status = snprintf(tmp, BUFFER_CAPACITY, "%.5f", argv.at(0));
        break;
      }
      case 1: {
        status = snprintf(tmp, BUFFER_CAPACITY, "%c", argv_2.at(0));
        break;
      }
      case 2: {
        status = snprintf(tmp, BUFFER_CAPACITY, "%.5f", argv.at(1));
        break;
      }
      case 3: {
        status = snprintf(tmp, BUFFER_CAPACITY, "%c", argv_2.at(1));
        break;
      }
      case 4: {
        status = snprintf(tmp, BUFFER_CAPACITY, "%.5f", argv.at(2));
        break;
      }
    }
    
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
