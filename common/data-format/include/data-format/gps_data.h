// @author vektor dewanto
#ifndef GPS_DATA_H
#define GPS_DATA_H

#include <string>
#include <stdio.h> // for snprintf()
#include <stdint.h> // for uint16_t, uint8_t, etc
#include <vector>

#include "data-format/string_data.h"

namespace crim {

static const int BUFFER_CAPACITY = 100;

struct GPSData: public StringData {
  GPSData();
  ~GPSData();

  void set_time(uint8_t hour, uint8_t minute, uint8_t second, uint16_t millisecond);
  void set_date(uint8_t year, uint8_t month, uint8_t day);
  void set_pose(float latitude, char lat, float longitude, char lon, float altitude);
  void set_misc(float angle, float speed);
  void set_note(bool fix, uint8_t fixquality, uint8_t n_satellite);
};

}// namespace crim

#endif
