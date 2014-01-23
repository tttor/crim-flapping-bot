#include "wirish.h"

#define PRESCALE    22
#define UPPERLIMIT  6545
#define LOWERLIMIT  3270

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
	init();
}   

void ESC(uint16 motorData)
{
  pwmWrite(D12, map(motorData,0, 1000, LOWERLIMIT, UPPERLIMIT));
}

void motorControl(uint16 motorData)    
{
  
  uint16 PWMData;

  if(motorData <= 0)  
    PWMData = 0; 
  else if(motorData >= 999) 
    PWMData = 50000; 
  else  
    PWMData = (1000 + motorData)*24;
    
  pwmWrite(D12,PWMData);
  
}

int main(void) {
  unsigned char data_count, i;
  unsigned int data;
  unsigned char num[5];
  char data_char;
  
  Serial1.begin(9600);
  
  pinMode(D12, PWM);
  
  Timer3.setPeriod(20000);
  //Timer3.setPrescaleFactor(PRESCALE);
  
  ESC(0);
  delay(500);
  
  
  while(1) {
    Serial1.print("Input PWM : ");
    
    i = 0;
    while(1) {
      
      if(Serial1.available()) {
        
        data_char = Serial1.read();
        Serial1.write(data_char);
        
        num[i] = (unsigned char) data_char;
        
        i++;
      }
      
      if(i >= 4) {
        
        data = num[0] - 0x30;
        data = data*10 + num[1] - 0x30;
        data = data*10 + num[2] - 0x30;
        
        Serial1.println();
        Serial1.println(data);
        
        if(data<1000 && data>0)
        {
          ESC(data);
          delay(100);
        }
        
        break;
        
      }
      delay(50);
      
      
    }
    
  }
  
  return 0;
  
}
