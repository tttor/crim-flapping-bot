#ifndef COMPASS_H
#define COMPASS_H

#include "sensor.h"

namespace crim {

class Compass : public Sensor {
 public:
	Compass();
	~Compass();
	
  void read();
  double x();
  double y();
  double z();
  
	std::vector<double> getReading();
	void calibrate();
  
 private:
	static const unsigned char kCompassAddress;
	static const short kSign[3];
	static const double kScale[3];
  static const unsigned char kGain;
  
  double x_, y_, z_;
  
  void getRawReading(short& x,short& y,short& z);
	
};

} // namespace crim
#endif
