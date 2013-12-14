#ifndef I2C_COMM_H
#define I2C_COMM_H

#include <cassert>
#include "wirish.h"
#include "i2c.h"

////////////////////////////////////////////////////////////////////////////////////
// Function prototype: void writeTo (uint8 DEVICE uint8 address, uint8 val)
// Parameter Description: DEVICE: I2C device address
// Address: Operation register address
// Val: write register values
// Return Value: None
// Description: val is written to the corresponding address register through the I2C bus
///////////////////////////////////////////////////////////////////////////////////

void writeTo(uint8 DEVICE, uint8 address, uint8 val) 
{
  // all i2c transactions send and receive arrays of i2c_msg objects 
  i2c_msg msgs[1]; // we dont do any bursting here, so we only need one i2c_msg object
  uint8 msg_data[2] = {address,val};
 
  msgs[0].addr = DEVICE;
  msgs[0].flags = 0; // write
  msgs[0].length = 2; //Write two data
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C1, msgs, 1,0);  //
}

////////////////////////////////////////////////////////////////////////////////////
// Function prototype: void readFrom (the uint8 DEVICE uint8 address, uint8 num, uint8 * msg_data)
// Parameter Description: DEVICE: I2C device address
// Address: Operation register address
// Num: the number of reads
// * Msg_data: read data stored pointer
// Return Value: None
// Description: I2C bus to read data
///////////////////////////////////////////////////////////////////////////////////

void readFrom(uint8 DEVICE, uint8 address, uint8 num, uint8 *msg_data) 
{
  i2c_msg msgs[1];
  msg_data[0] = address;
  
  msgs[0].addr = DEVICE;
  msgs[0].flags = 0; //write flag is 0
  msgs[0].length = 1; // just one byte for the address to read, 0x00
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C1, msgs, 1,0);
  
  msgs[0].addr = DEVICE;
  msgs[0].flags = I2C_MSG_READ; //read
  msgs[0].length = num; // Read the number of bytes
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C1, msgs, 1,0);
}


#endif
