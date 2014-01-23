#ifndef ESC_H
#define ESC_H

#include <stdint.h> // for uint16_t, uint8_t, etc
#include "wirish.h"

namespace crim {

class Esc {
 public:
  /**
  * @brief set the PWM frequency in hertz maximum 1MHz
  */
  Esc(uint8_t pwm_pin, uint64_t frequency);
  
  /**
   * @brief do nothing
  */
  ~Esc();
  
  /**
   * @brief set the speed
  */
  void set_speed(uint8_t duty_cycle);  
  
 private:
  
  uint8_t pwm_pin_;
  uint64_t frequency_;
};

}// namespace crim

#endif
