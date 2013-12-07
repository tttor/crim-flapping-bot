// @author : saripudin

#ifndef HMC5883_H
#define HMC5883_H

///////////////////////////HMC5883电子罗盘///////////////////////////////
#define HMC5883_ADDR 0x1E // 7 bit address of the HMC5883 used with the Wire library HMC5883 地址
#define HMC_POS_BIAS 1  //正基准值配置量
#define HMC_NEG_BIAS 2  //负基准值配置量

// HMC5883 寄存器定义
#define HMC5883_R_CONFA 0  //Configuration Register A   Read/Write 
#define HMC5883_R_CONFB 1  //Configuration Register B   Read/Write 
#define HMC5883_R_MODE 2   //Mode Register  Read/Write 
#define HMC5883_R_XM 3    //Data Output X MSB Register  Read
#define HMC5883_R_XL 4    //Data Output X LSB Register  Read
#define HMC5883_R_ZM 5    //Data Output Z MSB Register  Read 
#define HMC5883_R_ZL 6    //Data Output Z LSB Register  Read 
#define HMC5883_R_YM 7    //Data Output Y MSB Register  Read 
#define HMC5883_R_YL 8    //Data Output Y LSB Register  Read
#define HMC5883_R_STATUS 9 //Status Register  Read 
#define HMC5883_R_IDA 10   // Identification Register A  Read
#define HMC5883_R_IDB 11   //Identification Register B  Read  
#define HMC5883_R_IDC 12   //Identification Register C  Read

// Local magnetic declination
// I use this web : http://www.ngdc.noaa.gov/geomag-web/#declination
#define MAGNETIC_DECLINATION -6.0    // not used now -> magnetic bearing

float x_scale = 1;
float y_scale = 1;
float z_scale = 1;
float x_max,y_max,z_max;

void compassTest(void);
void compassInit(uint8 setmode);
void compassSetMode(uint8 mode);
void compassCalibrate(uint8 gain);
void compassSetDOR(uint8 DOR);
void compassSetGain(uint8 gain);
void getMagnetometerData(int16 * result);
double compassHeading(void);
void compassTest(void);


void compassInit(uint8 setmode)
{
  delay(5);
  if (setmode)
  {
    compassSetMode(0);
  }
 // writeTo(HMC5883_ADDR, HMC5883_R_CONFA, 0x70); //8-average, 15 Hz default, normal measurement 每次输出采样8次，15HZ输出采样率，普通模式
  //writeTo(HMC5883_ADDR, HMC5883_R_CONFB, 0xa0); // Gain=5, or any other desired gain 
  writeTo(HMC5883_ADDR, HMC5883_R_MODE, 0x00); // Set continouos mode (default to 10Hz)
}
void compassSetMode(uint8 mode) 
{ 
  if (mode > 2) 
  {
    return;
  }
  writeTo(HMC5883_ADDR, HMC5883_R_MODE, mode);
  delay(100);
}

void compassCalibrate(uint8 gain) 
{
  int16 compassdata[3]; 
  float fx = 0;
  float fy = 0;
  float fz = 0;
  x_scale = 1; // get actual values
  y_scale = 1;
  z_scale = 1;
  writeTo(HMC5883_ADDR, HMC5883_R_CONFA, 0x10 + HMC_POS_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to pos bias
  compassSetGain(gain);
  float x, y, z, mx=0, my=0, mz=0;
  
  for (uint8 i=0; i<10; i++) 
  { 
    compassSetMode(1);
    getMagnetometerData(compassdata); 
  
    fx = ((float) compassdata[0]) / x_scale;
    fy = ((float) compassdata[1]) / y_scale;  
    fz = ((float) compassdata[2]) / z_scale;  
    x= (int16) (fx + 0.5);
    y= (int16) (fy + 0.5);
    z= (int16) (fz + 0.5);
  
    if (x>mx) mx = x;
    if (y>my) my = y;
    if (z>mz) mz = z;
  }
  
  float maxi = 0;
  if (mx>maxi) maxi = mx;
  if (my>maxi) maxi = my;
  if (mz>maxi) maxi = mz;
  x_max = mx;
  y_max = my;
  z_max = mz;
  x_scale = maxi/mx; // calc scales
  y_scale = maxi/my;
  z_scale = maxi/mz;
  writeTo(HMC5883_ADDR, HMC5883_R_CONFA, 0x010); // set RegA/DOR back to default
}


// set data output rate
// 0-6, 4 default, normal operation assumed
void compassSetDOR(uint8 DOR) 
{
  if (DOR>6) return;
  writeTo(HMC5883_ADDR, HMC5883_R_CONFA, DOR<<2);
}


void compassSetGain(uint8 gain)
{ 
  // 0-7, 1 default
  if (gain > 7) return;
  writeTo(HMC5883_ADDR, HMC5883_R_CONFB, gain << 5);
}


void getMagnetometerData(int16 * result) 
{
  uint8 buff[6];
  
  readFrom(HMC5883_ADDR, HMC5883_R_XM, 6, buff);
  
  // MSB byte first, then LSB, X,Y,Z
  result[0] = (((int16)buff[0]) << 8) | buff[1];    // X axis 
  result[1] = (((int16)buff[4]) << 8) | buff[5];    // Y axis 
  result[2] = (((int16)buff[2]) << 8) | buff[3];    // Z axis
  
  result[0] = ( result[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
  result[1] = ( result[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
  result[2] = ( result[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
  
}

double compassHeading(void)
{
  float fx = 0;
  float fy = 0;
  float heading  = 0;
  int16 compassdata[3]; 
  getMagnetometerData(compassdata); 
  delay(67);//如果采样率为15HZ需要延迟67MS

  fx = ((float) compassdata[0]) / x_scale;
  fy = ((float) compassdata[1]) / y_scale;
  heading = atan2(fy, fx);
  
  // Correct for when signs are reversed.
  if(heading < 0)    heading += 2*PI;

  return(heading * 180/PI); 

}

void compassTest(void)//HMC5883罗盘测试
{
  while(1)
  {
    int16 res[3]; 
    getMagnetometerData(res); 
    //Print out values of each axis
    SerialUSB.print(res[0]);
    SerialUSB.print(" ");
    SerialUSB.print(res[1]);
    SerialUSB.print(" ");
    SerialUSB.println(res[2]);
    delay(20);
  }  
}
#endif
