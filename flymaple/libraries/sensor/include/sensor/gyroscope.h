#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include "sensor.h"

namespace crim {
class Gyroscope : public Sensor {
 private:
	static const unsigned char GyroAddress;
	static const unsigned char PWR_MGM;
	static const unsigned char DLPF_FS;
	static const unsigned char INT_CFG;
	static const unsigned char SMPLRT_DIV;
	static const unsigned char GYROX_H;
	static const short sign[3];
	static short offset[3];
	static const float sensitivity;
  
 public:
  Gyroscope();
	~Gyroscope();
  void getRawReading(short& x,short& y,short& z);
	Vector<double> getReading();
};

} // namespace crim

#endif
