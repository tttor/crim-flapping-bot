#ifndef BAROMETER_H
#define BAROMETER_H

#include "sensor.h"

namespace crim {

class Barometer : public Sensor {
 public:
  Barometer();
	~Barometer();
  
  void read();
  double getPressure();
  double getTemperature();
  double getAltitude();
  
	std::vector<double> getReading();
  
 private:
	static const unsigned char kBarometerAddress;
	static const unsigned char kOSS;
	static const unsigned char kBMP085_CAL_AC[6];
	static const unsigned char kBMP085_CAL_B[2];
	static const unsigned char kBMP085_CAL_MB;
	static const unsigned char kBMP085_CAL_MC;
	static const unsigned char kBMP085_CAL_MD;
	static const unsigned char kBMP085_CONTROL;
	static const unsigned char kBMP085_CONTROL_OUTPUT;
	static const unsigned char kREAD_TEMPERATURE;
	static const unsigned char kREAD_PRESSURE;
	static const int kMSLP;
	static const int kAltitude_cm_Offset;
  
	static short ac_[6],b_[2],mb_,mc_,md_;
	static int b5_;
  
  double temperature_, pressure_, altitude_;
  
  short read(unsigned char addr);
	void getRawReading(unsigned int& up,unsigned short& ut);
	int rawToBarometer(unsigned int up);
	short rawToTemperature(unsigned short ut);
	double pressureToAltitude(int p);
  
  

  
};

} // namespace crim

#endif
