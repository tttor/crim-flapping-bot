#ifndef SENSOR_H
#define SENSOR_H

//#include "matrix/vector.h"
#include <vector>

namespace crim {

class Sensor {
 public:
  ~Sensor();
  
 protected:
  Sensor();
  void read_i2c(unsigned char dev_addr,unsigned char read_addr,unsigned char read_length,unsigned char * buffer);
  void write_i2c(unsigned char dev_addr,unsigned char write_addr,unsigned char value);
  
};

} // namespace crim

#endif
