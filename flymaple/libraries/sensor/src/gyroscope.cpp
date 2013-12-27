#include "wirish.h"
#include "sensor/gyroscope.h"

using namespace crim;

const unsigned char Gyroscope::GyroAddress = 0x68;
const unsigned char Gyroscope::PWR_MGM = 0x3e;
const unsigned char Gyroscope::DLPF_FS = 0x16;
const unsigned char Gyroscope::INT_CFG = 0x17;
const unsigned char Gyroscope::SMPLRT_DIV = 0x15;
const unsigned char Gyroscope::GYROX_H = 0x1d;
const short Gyroscope::sign[3] = {1,1,1};
const float Gyroscope::sensitivity = 14.375;
short Gyroscope::offset[3] = {0,0,0};

Gyroscope::Gyroscope()
{
	write(GyroAddress,PWR_MGM,0x00); delay(1);	
	write(GyroAddress,SMPLRT_DIV,0x07);	
	write(GyroAddress,DLPF_FS,0x1e);	 delay(1);
	write(GyroAddress,INT_CFG, 0x00);			
	
	float accumulator[] = {0,0,0};
	for(int i = 0 ; i < 100 ; i++) {
		short x,y,z;
		getRawReading(x,y,z);
		accumulator[0] += x;
		accumulator[1] += y;
		accumulator[2] += z;
	}
	for(int i = 0 ; i < 3 ; i++) offset[i] = accumulator[i] / 100;
}

void Gyroscope::getRawReading(short& x,short& y,short& z)
{
	unsigned char buffer[6];
	read(GyroAddress,GYROX_H,6,buffer);
  x = (((short)buffer[0]) << 8) | buffer[1];    // X axis
  y = (((short)buffer[2]) << 8) | buffer[3];    // Y axis
  z = (((short)buffer[4]) << 8) | buffer[5];    // Z axis
}

Gyroscope::~Gyroscope()
{
}

Vector<double> Gyroscope::getReading()
{
	short x,y,z;
  
	getRawReading(x,y,z);
  
	x = sign[0] * (x - offset[0]);
	y = sign[1] * (y - offset[1]);
	z = sign[2] * (z - offset[2]);
	Vector<double> retVal(3);
	retVal(0) = (x * 1.0 / sensitivity) * (3.1415926 / 180);
	retVal(1) = (y * 1.0 / sensitivity) * (3.1415926 / 180);
	retVal(2) = (z * 1.0 / sensitivity) * (3.1415926 / 180);
	return retVal;
}
