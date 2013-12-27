#ifndef SENSOR_H
#define SENSOR_H

#include "matrix/vector.h"

namespace crim {

class Sensor {
 public:
  ~Sensor();
  
 protected:
  Sensor();
  void read(unsigned char dev_addr,unsigned char read_addr,unsigned char read_length,unsigned char * buffer);
  void write(unsigned char dev_addr,unsigned char write_addr,unsigned char value);
};

} // namespace crim

#endif
