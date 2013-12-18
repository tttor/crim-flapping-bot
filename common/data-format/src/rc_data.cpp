#include "data-format/rc_data.h"

using namespace crim;
using namespace std;

RCData::RCData(uint8_t n_ch) {
  content.resize(1+n_ch);
  content.at(0).push_back("RCS");// data id
  
  for (uint8_t i=1; i<content.size(); ++i) {
    content.at(i).resize(2);// 0-> init_PPM, 1-> PPM
  }
}

RCData::~RCData() {
  // nothing
}

void RCData::set_init_PPMs(const std::vector<uint16_t>& init_PPMs) {
  for (uint8_t i=0; i<init_PPMs.size(); ++i) {
    char tmp[BUFFER_CAPACITY];
    
    int status;
    status = snprintf(tmp, BUFFER_CAPACITY, "%d", init_PPMs.at(i));
    
    if ((status>0) && (status<=BUFFER_CAPACITY)) 
      content.at(i+1).at(0) = tmp;
  }
}

void RCData::set_PPMs(const std::vector<uint16_t>& PPMs) {
  for (uint8_t i=0; i<PPMs.size(); ++i) {
    char tmp[BUFFER_CAPACITY];
    
    int status;
    status = snprintf(tmp, BUFFER_CAPACITY, "%d", PPMs.at(i));
    
    if ((status>0) && (status<=BUFFER_CAPACITY)) 
      content.at(i+1).at(1) = tmp;
  }
}

