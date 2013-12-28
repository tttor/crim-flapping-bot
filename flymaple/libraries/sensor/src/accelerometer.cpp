#include "wirish.h"
#include "sensor/accelerometer.h"

using namespace crim;

const unsigned char Accelerometer::kAccelAddress = 0x53;
const unsigned char Accelerometer::kXL345_DEVID = 0xe5;
const unsigned char Accelerometer::kADXLREG_BW_RATE = 0x2c;
const unsigned char Accelerometer::kADXLREG_POWER_CTL = 0x2d;
const unsigned char Accelerometer::kADXLREG_DATA_FORMAT = 0x31;
const unsigned char Accelerometer::kADXLREG_DEVID = 0x00;
const unsigned char Accelerometer::kADXLREG_DATAX0 = 0x32;
const unsigned short Accelerometer::kGRAVITY = 248;
const short Accelerometer::kSign[3] = {1,1,-1};
const float Accelerometer::kSensitivity = 0.004;
short Accelerometer::offset_[3] = {0,0,0};

Accelerometer::Accelerometer() {
  unsigned char buffer[2];
  read_i2c(kAccelAddress,kADXLREG_DEVID,1,buffer);
  unsigned char dev_id = buffer[0];
  if(dev_id != kXL345_DEVID) {
    SerialUSB.println("Error, incorrect xl345 devid!");
    SerialUSB.println("Halting program, hit reset...");
    waitForButtonPress(0);
  }
  
  write_i2c(kAccelAddress,kADXLREG_POWER_CTL,0x00);        delay(5);     
  write_i2c(kAccelAddress,kADXLREG_POWER_CTL,0xff);        delay(5);  
  write_i2c(kAccelAddress,kADXLREG_POWER_CTL,0x08);        delay(5);
  write_i2c(kAccelAddress,kADXLREG_DATA_FORMAT,0x08);      delay(5);
  write_i2c(kAccelAddress,kADXLREG_BW_RATE,0x09);          delay(5);

  float accumulator[] = {0,0,0};
  for(int i = 0 ; i < 100 ; i++) {
    short x,y,z;
    getRawReading(x,y,z);
    accumulator[0] += x;
    accumulator[1] += y;
    accumulator[2] += z;
  }
  for(int i = 0 ; i < 3 ; i++) accumulator[i] /= 100;

  accumulator[2] -= kGRAVITY;
  for(int i = 0 ; i < 3 ; i++) offset_[i] = accumulator[i];
}

void Accelerometer::getRawReading(short& x,short& y,short& z) {
  unsigned char buffer[6];
  read_i2c(kAccelAddress,kADXLREG_DATAX0,6,buffer);
  x = (((short)buffer[1]) << 8) | buffer[0]; // X axis
  y = (((short)buffer[3]) << 8) | buffer[2]; // Y axis
  z = (((short)buffer[5]) << 8) | buffer[4]; // Z axis
}

Accelerometer::~Accelerometer() {
}

std::vector<double> Accelerometer::getReading() {
  short x,y,z;
  getRawReading(x,y,z);
  x = kSign[0] * (x - offset_[0]);
  y = kSign[1] * (y - offset_[1]);
  z = kSign[2] * (z - offset_[2]);
  
  std::vector<double> retVal(3);
  retVal.at(0) = x * kSensitivity;
  retVal.at(1) = y * kSensitivity;
  retVal.at(2) = z * kSensitivity;
  return retVal;
}

void Accelerometer::read() {
  std::vector<double> retVal = getReading();
  x_ = retVal.at(0);
  y_ = retVal.at(1);
  z_ = retVal.at(2);
}

double Accelerometer::x() {
  return x_;
}


double Accelerometer::y() {
  return y_;
}

double Accelerometer::z() {
  return z_;
}


