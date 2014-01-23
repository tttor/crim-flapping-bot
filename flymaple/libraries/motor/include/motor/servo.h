#ifndef SERVO_H
#define SERVO_H

#include <stdint.h> // for uint16_t, uint8_t, etc
#include "wirish.h"

namespace crim {

class Servo {
 public:
  /**
  * @brief set the PWM frequency in hertz maximum 1MHz
  */
  Servo(uint8_t pwm_pin);
  
  /**
   * @brief do nothing
  */
  ~Servo();
  
  /**
   * @brief set the speed
  */
  void set_angle(uint8_t angle);  
  
 private:
  
  uint8_t pwm_pin_;
};

}// namespace crim

#endif
