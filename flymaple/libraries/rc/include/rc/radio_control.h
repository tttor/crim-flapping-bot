// @author saripudin, vektor dewanto
#ifndef RADIO_CONTROL_H
#define RADIO_CONTROL_H

#include <stdint.h> // for uint16_t, uint8_t, etc
#include <vector>
#include "wirish.h"

namespace crim{

class RadioControl {
 public:
  typedef void (* void_mem_func_ptr_t)();
 
  /**
   * @brief This handles up to 8 channels RC
  */
  RadioControl(std::vector<uint8_t> ch_pins);
  
  /**
   * @brief
  */
  ~RadioControl();

  /**
   * @brief
  */  
  uint16 read(uint8_t ch);

  static const uint16_t kPPMThresholdBelow;
  static const uint16_t kPPMThresholdAbove;
  static const uint16_t kPPMTolerance;
  
 private:
  void set_ch_PPM(const uint8_t& pin, uint16_t* ch_begin, uint16_t* ch_end, uint16_t* ch_PPM);
  void ch_1_int_handler();
  void ch_2_int_handler();
  void ch_3_int_handler();
  void ch_4_int_handler();
  void ch_5_int_handler();
  void ch_6_int_handler();
  void ch_7_int_handler();
  void ch_8_int_handler();

  static std::vector<uint8_t> ch_pins_;  
  static std::vector<uint16_t> ch_PPMs_;
  static std::vector<uint16_t> ch_begins_;
  static std::vector<uint16_t> ch_ends_;
  std::vector<void_mem_func_ptr_t> ch_int_handlers_;
  uint8_t n_active_ch_;
};

}// namespace crim
#endif
