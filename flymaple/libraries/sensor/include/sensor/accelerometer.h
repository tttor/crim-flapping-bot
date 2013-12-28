#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include "sensor.h"

namespace crim {

class Accelerometer : public Sensor {
 
 public:
  Accelerometer();
  ~Accelerometer();
  
  void read();
  double x();
  double y();
  double z();
  
  std::vector<double> getReading();
  
 private:
  static const unsigned char kAccelAddress;
  static const unsigned char kXL345_DEVID;
  static const unsigned char kADXLREG_BW_RATE;
  static const unsigned char kADXLREG_POWER_CTL;
  static const unsigned char kADXLREG_DATA_FORMAT;
  static const unsigned char kADXLREG_DEVID;
  static const unsigned char kADXLREG_DATAX0;
  static const unsigned short kGRAVITY;
  static const short kSign[3];
  static const float kSensitivity;
  static short offset_[3];

  double x_, y_, z_;
  
  void getRawReading(short& x,short& y,short& z);
    
};

} // namespace crim

#endif

