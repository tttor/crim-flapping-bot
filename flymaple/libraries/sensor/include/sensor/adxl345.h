// @author : saripudin

#ifndef ADXL345_H
#define ADXL345_H

#include "i2c.h"
#include "i2c_comm.h"
#include "config_sensor.h"

void initAcc(void) ;
void getAccelerometerData(int16 * result);
void accelerometerTest(void);

#define ACC           (0x53)    // Defined ADXL345 address, ALT ADDRESS pin is grounded
#define A_TO_READ     (6)       // the number of bytes to read(each axis accounted for two-byte)
#define XL345_DEVID   0xE5      // ADXL345 ID register

// ADXL345 Control register
#define ADXLREG_TAP_AXES     0x2A
#define ADXLREG_BW_RATE      0x2C
#define ADXLREG_POWER_CTL    0x2D
#define ADXLREG_INT_ENABLE   0x2E
#define ADXLREG_DATA_FORMAT  0x31
#define ADXLREG_FIFO_CTL     0x38
#define ADXLREG_DUR          0x21

//ADXL345 Data register
#define ADXLREG_DEVID        0x00
#define ADXLREG_DATAX0       0x32
#define ADXLREG_DATAX1       0x33
#define ADXLREG_DATAY0       0x34
#define ADXLREG_DATAY1       0x35
#define ADXLREG_DATAZ0       0x36
#define ADXLREG_DATAZ1       0x37

// Accelerometer correction offset
int16 a_offset[3];

void initAcc(void) 
{
  
  unsigned char buffer[2];
  
  readFrom(ACC,ADXLREG_DEVID,1,buffer);

  unsigned char dev_id = buffer[0];
  
	if(dev_id != XL345_DEVID) {
		SerialUSB.println("Error, incorrect xl345 devid!");
		SerialUSB.println("Halting program, hit reset...");
		waitForButtonPress(0);
	}
  // default 100 Hz
  // +- 2g
  // resolusi dijaga yaitu 4mg/LSB
  
  writeTo(ACC, ADXLREG_POWER_CTL,0x00);
  //full resolution == 1
  writeTo(ACC, ADXLREG_DATA_FORMAT, 0x09); //0x0B-> +-16G : 0x09 -> +-2g
  writeTo(ACC, ADXLREG_BW_RATE, 0x0D); // 3.2 KHz -> 1 Set Low Power Mode
  writeTo(ACC, ADXLREG_POWER_CTL,0x08); //Set accelerometer to measure mode

  delay(100);
  
  // Calculate offset
  float accumulator[] = {0,0,0};
  int num_samples = 30;
  
  for(int i = 0 ; i < num_samples ; i++) {
    short acc[3];
    getAccelerometerData(acc);
    accumulator[0] += acc[0];
    accumulator[1] += acc[1];
    accumulator[2] += acc[2];
    delay(50);
  }
  
  for(int i = 0 ; i < 3 ; i++) accumulator[i] /= num_samples;
  accumulator[2] -= 256; // 1g at 2mg/LSB more or less.
  for(int i = 0 ; i < 3 ; i++) a_offset[i] = accumulator[i];
  
  for(int i = 0 ; i < 3 ; i++) SerialUSB.println(accumulator[i]);
}

void getAccelerometerData(int16 * result) {
  int16 regAddress = ADXLREG_DATAX0;    //start reading byte
  uint8 buff[A_TO_READ];

  readFrom(ACC, regAddress, A_TO_READ, buff); //read ADXL345 data and store it in buffer

  result[0] = ((((int16)buff[1]) << 8) | buff[0]) - a_offset[0];   
  result[1] = ((((int16)buff[3]) << 8) | buff[2]) - a_offset[1];
  result[2] = ((((int16)buff[5]) << 8) | buff[4]) - a_offset[2];
  
  result[0] = ( result[0] - ACCEL_X_OFFSET ) * ACCEL_X_SCALE;
  result[1] = ( result[1] - ACCEL_Y_OFFSET ) * ACCEL_Y_SCALE;
  result[2] = ( result[2] - ACCEL_Z_OFFSET ) * ACCEL_Z_SCALE;

}


void accelerometerTest(void) {
  int16 acc[3];
  while(1)
  {
    getAccelerometerData(acc);  //Read acceleration
    SerialUSB.print("Accel : ");
    SerialUSB.print(acc[0]);
    SerialUSB.print(" ");
    SerialUSB.print(acc[1]);
    SerialUSB.print(" ");
    SerialUSB.println(acc[2]);
    delay(20);
  }
}

#endif
