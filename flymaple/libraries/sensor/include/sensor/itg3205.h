// @author : saripudin

#ifndef ITG3205_H
#define ITG3205_H

#include "i2c.h"
#include "i2c_comm.h"
#include "config_sensor.h"

#define GYRO 0x68 
#define G_SMPLRT_DIV 0x15  
#define G_DLPF_FS 0x16 
#define G_INT_CFG 0x17
#define G_PWR_MGM 0x3E

#define G_TO_READ 8

int16 g_offx = 0;
int16 g_offy = 0;
int16 g_offz = 0;

void initGyro(void);
void zeroCalibrateGyroscope(uint16 totSamples, uint16 sampleDelayMS);
void getGyroscopeRaw(int16 * result);
void getGyroscopeData(int16 * result);
void GyroscopeTest(void);

void initGyro(void)
{
  writeTo(GYRO, G_PWR_MGM, 0x00);  //Internal oscillator
  writeTo(GYRO, G_SMPLRT_DIV, 0x07); // Fsample = 1kHz / (7 + 1) = 125Hz, or 8ms per sample.ITG3205 datasheet page 24    
  writeTo(GYRO, G_DLPF_FS, 0x1E);   //+/- 2000 dgrs/sec, 1KHz ,Low Pass Filter Bandwidth 5HZ
  
  writeTo(GYRO, G_INT_CFG, 0x00); 
}

void zeroCalibrateGyroscope(uint16 totSamples, uint16 sampleDelayMS) 
{
   //////////////////////////////////////
   // 陀螺仪ITG- 3205的I2C
   // 寄存器：
   // temp MSB = 1B, temp LSB = 1C
   // x axis MSB = 1D, x axis LSB = 1E
   // y axis MSB = 1F, y axis LSB = 20
   // z axis MSB = 21, z axis LSB = 22
   /////////////////////////////////////
  uint8 regAddress = 0x1D; // x axis MSB
  int16 xyz[3]; 
  float tmpOffsets[] = {0,0,0};
  uint8 buff[6];

  for (uint16 i = 0;i < totSamples;i++)
  {
    delay(sampleDelayMS);
    readFrom(GYRO, regAddress, 6, buff); //读取陀螺仪ITG3200 XYZ轴的数据
    xyz[0]= (((int16)buff[0] << 8) | buff[1]);
    xyz[1] = (((int16)buff[2] << 8) | buff[3]);
    xyz[2] = (((int16)buff[4] << 8) | buff[5]);
    tmpOffsets[0] += xyz[0];
    tmpOffsets[1] += xyz[1];
    tmpOffsets[2] += xyz[2];  
  }
  g_offx = -tmpOffsets[0] / totSamples;
  g_offy = -tmpOffsets[1] / totSamples;
  g_offz = -tmpOffsets[2] / totSamples;
}

void getGyroscopeRaw(int16 * result)
{
  uint8 regAddress = 0x1B;
  uint8 buff[G_TO_READ];

  readFrom(GYRO, regAddress, G_TO_READ, buff); //读取陀螺仪ITG3200的数据

  result[0] = (((int16)buff[2] << 8) | buff[3]) + g_offx;
  result[1] = (((int16)buff[4] << 8) | buff[5]) + g_offy;
  result[2] = (((int16)buff[6] << 8) | buff[7]) + g_offz;
  result[3] = ((int16)buff[0] << 8) | buff[1]; // 温度
}

void getGyroscopeData(int16 * result)
{
  int16 buff[4];
  getGyroscopeRaw(&buff[0]);  //读取原始数据
//  result[0] = buff[0];// / 14.375; // ITG3205 14.375  LSB/(º/s) 
//  result[1] = buff[1];// / 14.375;
//  result[2] = buff[2];// / 14.375;
  
  result[0] = buff[0];// / 14.375; // ITG3205 14.375  LSB/(º/s) 
  result[1] = buff[1];// / 14.375;
  result[2] = buff[2];// / 14.375;
}


void GyroscopeTest(void)  //ITG3205加速度读取测试例子
{
    int16 gyro[3];
    initGyro();           //初始化陀螺仪
    delay(1000);
    zeroCalibrateGyroscope(128,5);  //零值校正，记录陀螺仪静止状态输出的值将这个值保存到偏移量，采集128次，采样周期5ms
    while(1)
    {
      getGyroscopeData(gyro);    //读取陀螺仪      
      SerialUSB.print(gyro[0]);
      SerialUSB.print(" ");
      SerialUSB.print(gyro[1]);
      SerialUSB.print(" "); 
      SerialUSB.println(gyro[2]);
      delay(20);
    }
}



#endif
