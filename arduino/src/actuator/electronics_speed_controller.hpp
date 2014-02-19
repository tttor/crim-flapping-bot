// @author vektor dewanto
#ifndef ELECTRONICS_SPEED_CONTROLLER
#define ELECTRONICS_SPEED_CONTROLLER

#include <arduino-core/Arduino.h>

namespace crim {

/**
 * http://arduino.cc/en/Reference/AnalogWrite#.UwL-td_LXCQ
 * You do not need to call pinMode() to set the pin as an output before calling analogWrite().
 * The frequency of the PWM signal on most pins is approximately 490 Hz. 
 * On the Uno and similar boards, pins 5 and 6 have a frequency of approximately 980 Hz. Pins 3 and 11 on the Leonardo also run at 980 Hz.
 * On most Arduino boards (those with the ATmega168 or ATmega328), this function works on pins 3, 5, 6, 9, 10, and 11. 
 * On the Arduino Mega, it works on pins 2 - 13 and 44 - 46. Older Arduino boards with an ATmega8 only support analogWrite() on pins 9, 10, and 11.
 * 
 * Note that most ESCs accomodate only up to 50 Hz PWM.
 * Consider using the Servo arduino-lib; servo.attac()
 */
class ElectronicsSpeedController {
 public:
  ElectronicsSpeedController(uint8_t pwm_pin);
  ~ElectronicsSpeedController();
  
  /**
   * @brief set the speed i.e. the duty cycle,
   * \param percentage duty_cycle: 0.0 - 100.0 
  */
  void set_duty_cycle(double duty_cycle);  
  
  /**
   * @brief 
   * \param throtle: any rc channel value
   */
  void set_duty_cycle(uint16_t throtle);
  
  /**
   * @brief The default values are min= 1100 and max= 1900
   * 
   */
  void set_throtle_range(uint16_t min, uint16_t max);
  
 private:
  uint8_t pwm_pin_;
  uint16_t min_throtle_;
  uint16_t max_throtle_;
};
  
}// namespace crim

#endif
