#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include "sensor.h"

namespace crim {
class Gyroscope : public Sensor {
  
 public:
  Gyroscope();
	~Gyroscope();
  
  void read();
  double x();
  double y();
  double z();
  
	std::vector<double> getReading();
 private:
	static const unsigned char kGyroAddress;
	static const unsigned char kPWR_MGM;
	static const unsigned char kDLPF_FS;
	static const unsigned char kINT_CFG;
	static const unsigned char kSMPLRT_DIV;
	static const unsigned char kGYROX_H;
	static const short kSign[3];
	static short offset_[3];
	static const float kSensitivity;
  
  double x_, y_, z_;
  
  void getRawReading(short& x,short& y,short& z);
  

};

} // namespace crim

#endif
