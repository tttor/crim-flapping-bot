#include "wirish.h"
#include "sensor/gyroscope.h"

using namespace crim;

const unsigned char Gyroscope::kGyroAddress = 0x68;
const unsigned char Gyroscope::kPWR_MGM = 0x3e;
const unsigned char Gyroscope::kDLPF_FS = 0x16;
const unsigned char Gyroscope::kINT_CFG = 0x17;
const unsigned char Gyroscope::kSMPLRT_DIV = 0x15;
const unsigned char Gyroscope::kGYROX_H = 0x1d;
const short Gyroscope::kSign[3] = {1,1,1};
const float Gyroscope::kSensitivity = 14.375;
short Gyroscope::offset_[3] = {0,0,0};

Gyroscope::Gyroscope()
{
	write_i2c(kGyroAddress,kPWR_MGM,0x00); delay(1);	
	write_i2c(kGyroAddress,kSMPLRT_DIV,0x07);	
	write_i2c(kGyroAddress,kDLPF_FS,0x1e);	 delay(1);
	write_i2c(kGyroAddress,kINT_CFG, 0x00);			
	
	float accumulator[] = {0,0,0};
	for(int i = 0 ; i < 100 ; i++) {
		short x,y,z;
		getRawReading(x,y,z);
		accumulator[0] += x;
		accumulator[1] += y;
		accumulator[2] += z;
	}
	for(int i = 0 ; i < 3 ; i++) offset_[i] = accumulator[i] / 100;
}

void Gyroscope::getRawReading(short& x,short& y,short& z)
{
	unsigned char buffer[6];
	read_i2c(kGyroAddress,kGYROX_H,6,buffer);
  x = (((short)buffer[0]) << 8) | buffer[1];    // X axis
  y = (((short)buffer[2]) << 8) | buffer[3];    // Y axis
  z = (((short)buffer[4]) << 8) | buffer[5];    // Z axis
}

Gyroscope::~Gyroscope()
{
}

std::vector<double> Gyroscope::getReading()
{
	short x,y,z;
	getRawReading(x, y, z);
	x = kSign[0] * (x - offset_[0]);
	y = kSign[1] * (y - offset_[1]);
	z = kSign[2] * (z - offset_[2]);
	std::vector<double> retVal(3);
  
	retVal.at(0) = (x * 1.0 / kSensitivity) * (3.1415926 / 180);
	retVal.at(1) = (y * 1.0 / kSensitivity) * (3.1415926 / 180);
	retVal.at(2) = (z * 1.0 / kSensitivity) * (3.1415926 / 180);
	return retVal;
}


void Gyroscope::read() {
  std::vector<double> retVal = getReading();
  x_ = retVal.at(0);
  y_ = retVal.at(1);
  z_ = retVal.at(2);
}

double Gyroscope::x() {
  return x_;
}


double Gyroscope::y() {
  return y_;
}

double Gyroscope::z() {
  return z_;
}
