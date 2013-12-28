#include "wirish.h"
#include "sensor/barometer.h"

using namespace crim;

const unsigned char Barometer::kBarometerAddress = 0x77;
const unsigned char Barometer::kOSS = 0;
const unsigned char Barometer::kBMP085_CAL_AC[6] = {0xaa,0xac,0xae,0xb0,0xb2,0xb4};
const unsigned char Barometer::kBMP085_CAL_B[2] = {0xb6,0xb8};
const unsigned char Barometer::kBMP085_CAL_MB = 0xba;
const unsigned char Barometer::kBMP085_CAL_MC = 0xbc;
const unsigned char Barometer::kBMP085_CAL_MD = 0xbe;
const unsigned char Barometer::kBMP085_CONTROL = 0xf4;
const unsigned char Barometer::kBMP085_CONTROL_OUTPUT = 0xf6;
const unsigned char Barometer::kREAD_TEMPERATURE = 0x2e;
const unsigned char Barometer::kREAD_PRESSURE = 0x34;
const int Barometer::kMSLP = 101325;
const int Barometer::kAltitude_cm_Offset = 0;

short Barometer::ac_[6],Barometer::b_[2],Barometer::mb_,Barometer::mc_,Barometer::md_;
int Barometer::b5_;

Barometer::Barometer() {
	for(int i = 0 ; i < 6 ; i++) ac_[i] = read(kBMP085_CAL_AC[i]);
	for(int i = 0 ; i < 2 ; i++) b_[i] = read(kBMP085_CAL_B[i]);
	mb_ = read(kBMP085_CAL_MB);
	mc_ = read(kBMP085_CAL_MC);
	md_ = read(kBMP085_CAL_MD);
}

Barometer::~Barometer() {
}

std::vector<double> Barometer::getReading() {
	unsigned int up; unsigned short ut;
	getRawReading(up,ut);
	int pres = rawToBarometer(up);
	short temp = rawToTemperature(ut);
	double altitude = pressureToAltitude(pres);
	std::vector<double> retVal(3);
	retVal.at(0) = pres; 
  retVal.at(1) = temp; 
  retVal.at(2) = altitude;
	return retVal;
}

short Barometer::read(unsigned char addr) {
	unsigned char buffer[2];
	read_i2c(kBarometerAddress,addr,2,buffer);
	return ((((short)buffer[0]) << 8) | buffer[1]);
}

void Barometer::getRawReading(unsigned int& up,unsigned short& ut) {
	unsigned char buffer[3];
	up = 0;
	write_i2c(kBarometerAddress,kBMP085_CONTROL,(kREAD_PRESSURE + (kOSS << 6)));
	delay(2 + (3 << kOSS));
	read_i2c(kBarometerAddress,kBMP085_CONTROL_OUTPUT,3,buffer);
	up = (((unsigned int) buffer[0] << 16) | ((unsigned int) buffer[1] << 8) | (unsigned int) buffer[2]) >> (8-kOSS);
	write_i2c(kBarometerAddress,kBMP085_CONTROL,kREAD_TEMPERATURE);
	delay(5);
	read_i2c(kBarometerAddress,kBMP085_CONTROL_OUTPUT,2,buffer); 
	ut = ((((short)buffer[0]) << 8) | buffer[1]);
}

int Barometer::rawToBarometer(unsigned int up) {
    int x1, x2, x3, b3, b6, p;
    unsigned int b4, b7;
  
    b6 = b5_ - 4000;
    // Calculate B3
    x1 = (b_[1] * (b6 * b6)>>12)>>11;
    x2 = (ac_[1] * b6)>>11;
    x3 = x1 + x2;
    b3 = (((((int)ac_[0])*4 + x3)<<kOSS) + 2)>>2;
  
    // Calculate B4
    x1 = (ac_[2] * b6)>>13;
    x2 = (b_[0] * ((b6 * b6)>>12))>>16;
    x3 = ((x1 + x2) + 2)>>2;
    b4 = (ac_[3] * (unsigned int)(x3 + 32768))>>15;
  
    b7 = ((unsigned int)(up - b3) * (50000>>kOSS));
    if (b7 < 0x80000000)
        p = (b7<<1)/b4;
    else
        p = (b7/b4)<<1;
    
    x1 = (p>>8) * (p>>8);
    x1 = (x1 * 3038)>>16;
    x2 = (-7357 * p)>>16;
    p += (x1 + x2 + 3791)>>4;
  
    return p;
}

short Barometer::rawToTemperature(unsigned short ut) {
    int x1, x2;
  
    x1 = (((int)ut - (int)ac_[5])*(int)ac_[4]) >> 15;
    x2 = ((int)mc_ << 11)/(x1 + md_);
    b5_ = x1 + x2;

    return ((b5_ + 8)>>4); 
}

double Barometer::pressureToAltitude(int pressure) {
	double meters =  (44330 * (1 - pow(((float)pressure / (float)kMSLP), 0.1903))) + kAltitude_cm_Offset;
	return meters;
}



void Barometer::read() {
  std::vector<double> retVal = getReading();
  pressure_ = retVal.at(0);
  temperature_ = retVal.at(1);
  altitude_ = retVal.at(2);
}

double Barometer::getPressure() {
  return pressure_;
}


double Barometer::getTemperature() {
  return temperature_;
}

double Barometer::getAltitude() {
  return altitude_;
}
