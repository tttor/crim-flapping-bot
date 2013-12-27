#ifndef PRESSURE_H
#define PRESSURE_H

#include "sensor.h"

namespace crim {

class Pressure : public Sensor {
 private:
	static const unsigned char PressureAddress;
	static const unsigned char OSS;
	static const unsigned char BMP085_CAL_AC[6];
	static const unsigned char BMP085_CAL_B[2];
	static const unsigned char BMP085_CAL_MB;
	static const unsigned char BMP085_CAL_MC;
	static const unsigned char BMP085_CAL_MD;
	static const unsigned char BMP085_CONTROL;
	static const unsigned char BMP085_CONTROL_OUTPUT;
	static const unsigned char READ_TEMPERATURE;
	static const unsigned char READ_PRESSURE;
	static const int MSLP;
	static const int Altitude_cm_Offset;
	static short ac[6],b[2],mb,mc,md;
	static int b5;
	
 public:
  Pressure();
	~Pressure();
	Vector<double> getReading();
  short readP(unsigned char addr);
	void getRawReading(unsigned int& up,unsigned short& ut);
	int rawToPressure(unsigned int up);
	short rawToTemperature(unsigned short ut);
	double pressureToAltitude(int p);
};

} // namespace crim

#endif
