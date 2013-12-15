// @author hendruw lim, vektor dewanto
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h> // for uint16_t, uint8_t, etc
#include "wirish.h"

namespace crim {

class Motor {
 public:
  /**
  * @brief set the PWM frequency in hertz maximum 1MHz
  */
  Motor(uint8_t pwm_pin, uint8_t dir_1_pin, uint8_t dir_2_pin, uint64_t frequency);
  
  /**
   * @brief do nothing
  */
  ~Motor();
  
  /**
   * @brief 
  */
  void forward(uint8_t duty_cycle);
  
  /**
   * @brief 
  */
  void backward(uint8_t duty_cycle);

  /**
   * @brief 
  */
  void stop(uint8_t type=0);  
  
 private:
  /**
   * @brief set the PWM duty cycle in percentage from 0-100
  */
  void set_duty_cycle(uint8_t duty_cycle);  
  
  uint8_t pwm_pin_;
  uint8_t dir_1_pin_;
  uint8_t dir_2_pin_;
  uint64_t frequency_;
};

}// namespace crim

#endif
