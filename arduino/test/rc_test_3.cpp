#include <arduino-core/Arduino.h>
#include <comm/radio_control.hpp>

volatile uint16_t ch_values_[4] = {0};
volatile uint16_t ch_timer_begins_[4] = {0};

uint8_t ch_pins_[4];
voidFuncPtr ch_int_handlers_[4];

uint16_t read(uint8_t ch) {
  return ch_values_[ch-1];
}

void update_ch_value(const uint8_t& pin, volatile uint16_t* ch_timer_begin, volatile uint16_t* ch_value) {
  if (digitalRead(pin) == 1) {
    *ch_timer_begin = micros();
  } 
  else {
    if (*ch_timer_begin != 0) {
      *ch_value = micros() - *ch_timer_begin;
      *ch_timer_begin = 0;
    }
  }   
}  

void ch_1_int_handler() {
  update_ch_value(ch_pins_[0], &ch_timer_begins_[0], &ch_values_[0]);
}

void ch_2_int_handler() {
  update_ch_value(ch_pins_[1], &ch_timer_begins_[1], &ch_values_[1]);
}

void ch_3_int_handler() {
  update_ch_value(ch_pins_[2], &ch_timer_begins_[2], &ch_values_[2]);
}

void ch_4_int_handler() {
  update_ch_value(ch_pins_[3], &ch_timer_begins_[3], &ch_values_[3]);
}


int main() {
  init();// this needs to be called before setup() or some functions won't work there
  Serial.begin(115200);
  
  const uint8_t kNChannel = 4;
  uint8_t n_ch_ = kNChannel;
  
  ch_pins_[0] = 21;
  ch_pins_[1] = 20;
  ch_pins_[2] = 19;
  ch_pins_[3] = 18;
  
  ch_int_handlers_[0] = (voidFuncPtr) &ch_1_int_handler;
  ch_int_handlers_[1] = (voidFuncPtr) &ch_2_int_handler;
  ch_int_handlers_[2] = (voidFuncPtr) &ch_3_int_handler;
  ch_int_handlers_[3] = (voidFuncPtr) &ch_4_int_handler;
  
  for (uint8_t i=0; i<n_ch_; ++i) {
    pinMode(ch_pins_[i], INPUT);
    attachInterrupt(i+2, ch_int_handlers_[i], CHANGE);// plus two as we use external int2, int3, int4, int5
  }
  
  while (true) {
    for (uint8_t i=0; i<kNChannel; ++i) {
      Serial.println(read(i+1), DEC);
    }
    Serial.println("");
    
    delay(100);
  }
   
  Serial.end();
  return 0;
}
