#include "wirish.h"
#include "sensor/compass.h"

using namespace crim;

const unsigned char Compass::kCompassAddress = 0x1e;
const short Compass::kSign[3] = {1,1,1};
const double Compass::kScale[3] = {1,1.09,1};
const unsigned char Compass::kGain = 5;

Compass::Compass() {
	write_i2c(kCompassAddress,0x02,0x00);
	write_i2c(kCompassAddress,0x01,kGain << 5);
	write_i2c(kCompassAddress,0x00,0x70);			
	delay(100);
}

void Compass::getRawReading(short& x,short& y,short& z) {
	unsigned char buffer[6];
	read_i2c(kCompassAddress,0x03,6,buffer);
  x = (((short)buffer[0]) << 8) | buffer[1];    // X axis
  y = (((short)buffer[4]) << 8) | buffer[5];    // Y axis
  z = (((short)buffer[2]) << 8) | buffer[3];    // Z axis
}

Compass::~Compass()
{
}

std::vector<double> Compass::getReading() {
	short x,y,z;
	getRawReading(x,y,z);
	x = kSign[0] * x * kScale[0];
	y = kSign[1] * y * kScale[1];
	z = kSign[2] * z * kScale[2];
	std::vector<double> retVal(3);
	retVal.at(0) = x;	
  retVal.at(1) = y;	
  retVal.at(2) = z;
	return retVal;
}

void Compass::calibrate() {
  
	static double maxval, maxx, maxy, maxz;
	static bool isFirstCalibrate = true;
	if(isFirstCalibrate) {
		maxval = maxx = maxy = maxz = 0;
		isFirstCalibrate = false;
	}
	write_i2c(kCompassAddress,0x00,0x11);
	write_i2c(kCompassAddress,0x01,kGain << 5);
  
	for(int i = 0 ; i < 10 ; i++) {
		write_i2c(kCompassAddress,0x02,1);
		delay(100);
		short x,y,z;
		getRawReading(x,y,z);
		if(x > maxx) maxx = x; if(maxx > maxval) maxval = maxx;
		if(y > maxy) maxy = y; if(maxy > maxval) maxval = maxy;
		if(z > maxz) maxz = z; if(maxz > maxval) maxval = maxz;
	}
  
	write_i2c(kCompassAddress,0x00,0x70);
	write_i2c(kCompassAddress,0x02,0);
	delay(100);
}

void Compass::read() {
  std::vector<double> retVal = getReading();
  x_ = retVal.at(0);
  y_ = retVal.at(1);
  z_ = retVal.at(2);
}

double Compass::x() {
  return x_;
}


double Compass::y() {
  return y_;
}

double Compass::z() {
  return z_;
}

