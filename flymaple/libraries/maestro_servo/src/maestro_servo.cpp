#include "maestro_servo/maestro_servo.h"

void maestroSetServo(int ServoChannel, int target)
{
   byte serialBytes[6]; //Create the byte array object that will hold the communication packet.
 
   target = (map(target, 0, 180, 2400, 9600)); //Map the target angle to the corresponding PWM pulse. range : 600-2400 uS
 
   serialBytes[0] = 0xAA; // Command byte: Set Target.
   serialBytes[1] = 0x0C;
   serialBytes[2] = 0x04;
   serialBytes[3] = ServoChannel; // First byte holds channel number.
   serialBytes[4] = target & 0x7F; // Second byte holds the lower 7 bits of target.
   serialBytes[5] = (target >> 7) & 0x7F; // Third byte holds the bits 7-13 of target.
   
   Serial2.write(serialBytes, sizeof(serialBytes)); //Write the byte array to the serial port.
}

void maestroInit() {
    Serial2.begin(9600);
  
}
