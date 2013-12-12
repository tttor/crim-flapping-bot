// Test read sensor
#include "wirish.h"
#include "i2c.h"
#include "sensor/adxl345.h"
#include "sensor/itg3205.h"
#include "sensor/hmc5883.h"
#include "sensor/bmp085.h"

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
  int16 acc[3];
  int16 gyro[3];
  int16 mag[3];

  int16 temperature = 0;
  int32 pressure = 0;
  int32 centimeters = 0;
  
  i2c_master_enable(I2C1, I2C_FAST_MODE);
  delay(200);
  
  initAcc();
  delay(200);
  initGyro();
  delay(200);
  zeroCalibrateGyroscope(128,5);

  compassInit(false);  
  compassCalibrate(1);  
  compassSetMode(0);

  bmp085Calibration();
  
  while(1) {
    getAccelerometerData(acc);  //Read acceleration
    SerialUSB.print("Accel: ");
    SerialUSB.print(acc[0]);
    SerialUSB.print(" ");
    SerialUSB.print(acc[1]);
    SerialUSB.print(" ");
    SerialUSB.print(acc[2]);

    getGyroscopeData(gyro);  //Read acceleration
    SerialUSB.print(" Gyro: ");
    SerialUSB.print(gyro[0]);
    SerialUSB.print(" ");
    SerialUSB.print(gyro[1]);
    SerialUSB.print(" ");
    SerialUSB.print(gyro[2]);

    getMagnetometerData(mag);  //Read acceleration
    SerialUSB.print(" Mag: ");
    SerialUSB.print(mag[0]);
    SerialUSB.print(" ");
    SerialUSB.print(mag[1]);
    SerialUSB.print(" ");
    SerialUSB.print(mag[2]);

    temperature = bmp085GetTemperature(bmp085ReadUT());
    pressure = bmp085GetPressure(bmp085ReadUP());
    centimeters = bmp085GetAltitude();

    SerialUSB.print(" Temp: ");
    SerialUSB.print(temperature, DEC);
    SerialUSB.print(" *0.1 deg C ");
    SerialUSB.print("Pressure: ");
    SerialUSB.print(pressure, DEC);
    SerialUSB.print(" Pa ");
    SerialUSB.print("Altitude: ");
    SerialUSB.print(centimeters, DEC);
    SerialUSB.print(" cm ");
    SerialUSB.println("    ");
    
    delay(100);
  }
  
  return 0;
}
