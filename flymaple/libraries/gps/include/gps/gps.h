// @author vektor dewanto
#ifndef GPS_H
#define GPS_H

#include <stdlib.h> // for Adafruit_GPS lib, and this _must_ be on top of all #include
#include "wirish.h"
#include "gps/Adafruit_GPS.h"
#include "data-format/gps_data.h"

namespace crim {

class GlobalPositioningSystem {
 public:
  typedef void (* void_mem_func_ptr_t)();
  
  HardwareTimer* timer;// public as we need to attach timer interrupt routine outside
     
  GlobalPositioningSystem(HardwareSerial* serial, uint32_t baud_rate, uint8_t timer_num, uint64_t frequency);
  ~GlobalPositioningSystem();
  
  /**
   * @brief get the status of adafruit_GPS_->newNMEAreceived() and adafruit_GPS_->parse(adafruit_GPS_->lastNMEA())
   * 
  */ 
  bool status();
  
  /**
   * @ brief
   * 
  */
  void set_data(crim::GPSData* GPS_data);
  
  /**
   * @ brief
   * 
  */
  void read();
  
 private:
  Adafruit_GPS* adafruit_GPS_;
  
  /**
   * @ brief
   * 
  */
  void setup_adafruit_gps(HardwareSerial* serial, uint32_t baud_rate);
  
  /**
   * @brief This setup method is incomplete in that we have to set 
   * the imer->attachCompare1Interrupt(timer_int_handler) _outside_
   * TODO (@tttor) Hide this timer interrupt related methods inside the GlobalPositioning class
   * So far, any attempts failed; causing onboard-led strobing
  */ 
  void setup_timer(uint8_t number, uint64_t frequency);
};

}// namespace crim
#endif
