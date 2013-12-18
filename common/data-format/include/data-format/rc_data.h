// @author vektor dewanto
#ifndef RC_DATA_H
#define RC_DATA_H

#include <string>
#include <stdio.h> // for snprintf()
#include <stdint.h> // for uint16_t, uint8_t, etc
#include <vector>

#include "data-format/string_data.h"

namespace crim {

static const int BUFFER_CAPACITY = 100;

struct RCData: public StringData {
  RCData(uint8_t n_ch);
  ~RCData();
  
  void set_init_PPMs(const std::vector<uint16_t>& init_PPMs);
  void set_PPMs(const std::vector<uint16_t>& PPMs);
};

}// namespace crim

#endif
