#ifndef COMPASS_H
#define COMPASS_H

#include "sensor.h"

namespace crim {

class Compass : public Sensor {
 private:
	static Compass compass;
	static const unsigned char CompassAddress;
	static const short sign[3];
	static const double scale[3];
	
 public:
	Compass(unsigned char gain = 5);
	~Compass();
	void getRawReading(short& x,short& y,short& z);
	Vector<double> getReading();
	static void calibrate(unsigned char gain = 5);
};

} // namespace crim
#endif
