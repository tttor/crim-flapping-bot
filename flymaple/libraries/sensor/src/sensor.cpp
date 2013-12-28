#include <cassert>
#include "i2c.h"
#include "wirish.h"
#include "sensor/sensor.h"

using namespace crim;

Sensor::Sensor() {
}

Sensor::~Sensor() {
}

void Sensor::read_i2c(unsigned char dev_addr,unsigned char read_addr,unsigned char read_length,unsigned char * buffer) {
#ifndef NDEBUG
  assert(buffer);
#endif

  buffer[0] = read_addr;
  i2c_msg msgs[1];
  msgs[0].addr = dev_addr;
  msgs[0].flags = 0;
  msgs[0].length = 1;
  msgs[0].data = buffer;
  i2c_master_xfer(I2C1,msgs,1,0);

  msgs[0].addr = dev_addr;
  msgs[0].flags = I2C_MSG_READ;
  msgs[0].length = read_length;
  msgs[0].data = buffer;
  i2c_master_xfer(I2C1,msgs,1,0);
}

void Sensor::write_i2c(unsigned char dev_addr,unsigned char write_addr,unsigned char value) {
  unsigned char buffer[] = {write_addr,value};
  i2c_msg msgs[1];
  msgs[0].addr = dev_addr;
  msgs[0].flags = 0;
  msgs[0].length = 2;
  msgs[0].data = buffer;
  i2c_master_xfer(I2C1,msgs,1,0);
}

