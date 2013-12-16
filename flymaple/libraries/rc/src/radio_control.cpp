#include "rc/radio_control.h"

using namespace crim;

const uint16_t RadioControl::kPPMTresholdBelow = 950;
const uint16_t RadioControl::kPPMTresholdAbove = 2000;
  
std::vector<uint16_t> RadioControl::ch_PPMs_ = std::vector<uint16_t>();
std::vector<uint16_t> RadioControl::ch_begins_ = std::vector<uint16_t>();
std::vector<uint16_t> RadioControl::ch_ends_ = std::vector<uint16_t>();
std::vector<uint8_t> RadioControl::ch_pins_ = std::vector<uint8_t>();

RadioControl::RadioControl(std::vector<uint8_t> ch_pins) {
  //
  ch_pins_ = ch_pins;
  n_active_ch_ = ch_pins_.size();
  
  ch_PPMs_.resize(n_active_ch_);
  ch_begins_.resize(n_active_ch_);
  ch_ends_.resize(n_active_ch_);
  
  ch_int_handlers_.resize(8);// TODO (@tttor): Look for more elegant way :)
  ch_int_handlers_.at(0) = (void_mem_func_ptr_t) &RadioControl::ch_1_int_handler;
  ch_int_handlers_.at(1) = (void_mem_func_ptr_t) &RadioControl::ch_2_int_handler;
  ch_int_handlers_.at(2) = (void_mem_func_ptr_t) &RadioControl::ch_3_int_handler;
  ch_int_handlers_.at(3) = (void_mem_func_ptr_t) &RadioControl::ch_4_int_handler;
  ch_int_handlers_.at(4) = (void_mem_func_ptr_t) &RadioControl::ch_5_int_handler;
  ch_int_handlers_.at(5) = (void_mem_func_ptr_t) &RadioControl::ch_6_int_handler;
  ch_int_handlers_.at(6) = (void_mem_func_ptr_t) &RadioControl::ch_7_int_handler;
  ch_int_handlers_.at(7) = (void_mem_func_ptr_t) &RadioControl::ch_8_int_handler;

  //      
  for (uint8_t i=0; i<n_active_ch_; ++i) {
    pinMode(ch_pins_.at(i), INPUT);
    attachInterrupt(ch_pins_.at(i), ch_int_handlers_.at(i), CHANGE);
  } 
}
  
RadioControl::~RadioControl() {
}

uint16_t RadioControl::read(uint8_t ch) {
  return ch_PPMs_.at(ch-1);
}

void RadioControl::set_ch_PPM(const uint8_t& pin, uint16_t* ch_begin, uint16_t* ch_end, uint16_t* ch_PPM) {
  uint16_t delta = 0;
  
  if (digitalRead(pin) == 1) {
    *ch_begin = micros();
  } else {
    if (*ch_begin != 0) {
      *ch_end = micros();
      delta = *ch_end - *ch_begin;
      
      if((delta > kPPMTresholdBelow) && (delta < kPPMTresholdAbove)) {
        *ch_PPM = delta;
      }
      
      *ch_begin = 0;
    }
  }   
}  

void RadioControl::ch_1_int_handler() {
  set_ch_PPM(ch_pins_.at(0), &ch_begins_.at(0), &ch_ends_.at(0), &ch_PPMs_.at(0));
}

void RadioControl::ch_2_int_handler() {
  set_ch_PPM(ch_pins_.at(1), &ch_begins_.at(1), &ch_ends_.at(1), &ch_PPMs_.at(1));
}

void RadioControl::ch_3_int_handler() {
  set_ch_PPM(ch_pins_.at(2), &ch_begins_.at(2), &ch_ends_.at(2), &ch_PPMs_.at(2));
}

void RadioControl::ch_4_int_handler() {
  set_ch_PPM(ch_pins_.at(3), &ch_begins_.at(3), &ch_ends_.at(3), &ch_PPMs_.at(3));
}

void RadioControl::ch_5_int_handler() {
  set_ch_PPM(ch_pins_.at(4), &ch_begins_.at(4), &ch_ends_.at(4), &ch_PPMs_.at(4));
}

void RadioControl::ch_6_int_handler() {
  set_ch_PPM(ch_pins_.at(5), &ch_begins_.at(5), &ch_ends_.at(5), &ch_PPMs_.at(5));
}

void RadioControl::ch_7_int_handler() {
  set_ch_PPM(ch_pins_.at(6), &ch_begins_.at(6), &ch_ends_.at(6), &ch_PPMs_.at(6));
}

void RadioControl::ch_8_int_handler() {
  set_ch_PPM(ch_pins_.at(7), &ch_begins_.at(7), &ch_ends_.at(7), &ch_PPMs_.at(7));
}
