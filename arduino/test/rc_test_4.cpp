#include <arduino-core/Arduino.h>
//#include <comm/radio_control.hpp>

#include <arduino-core/Arduino.h>
#include <stdint.h> // for uint16_t, uint8_t, etc
#include <arduino-core/wiring_private.h>// for voidFuncPtr

namespace crim {

bool rc_init();
uint16_t rc_read(uint8_t ch);

void rc_ch_1_int_handler();
void rc_ch_2_int_handler();
void rc_ch_3_int_handler();
void rc_ch_4_int_handler();
void rc_update_ch_value(const uint8_t& pin, volatile uint16_t* ch_timer_begin, volatile uint16_t* ch_value);

volatile uint16_t g_rc_ch_values[4] = {0};
volatile uint16_t g_rc_ch_timer_begins[4] = {0};

uint8_t g_rc_ch_pins[4] = {21, 20, 19, 18};
voidFuncPtr g_rc_ch_int_handlers[4] = {(voidFuncPtr) &rc_ch_1_int_handler, (voidFuncPtr) &rc_ch_2_int_handler, (voidFuncPtr) &rc_ch_3_int_handler, (voidFuncPtr) &rc_ch_4_int_handler};

bool rc_init(uint8_t n_ch) {
  const uint8_t kMaxNChannel = 4;
  if (n_ch < kMaxNChannel) 
    return false;
  
  for (uint8_t i=0; i<n_ch; ++i) {
    pinMode(g_rc_ch_pins[i], INPUT);
    attachInterrupt(i+2, g_rc_ch_int_handlers[i], CHANGE);// plus two as we use external int2, int3, int4, int5
  }
  
  return true;
}

uint16_t rc_read(uint8_t ch) {
  return g_rc_ch_values[ch-1];
}

void rc_update_ch_value(const uint8_t& pin, volatile uint16_t* ch_timer_begin, volatile uint16_t* ch_value) {
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

void rc_ch_1_int_handler() {
  rc_update_ch_value(g_rc_ch_pins[0], &g_rc_ch_timer_begins[0], &g_rc_ch_values[0]);
}

void rc_ch_2_int_handler() {
  rc_update_ch_value(g_rc_ch_pins[1], &g_rc_ch_timer_begins[1], &g_rc_ch_values[1]);
}

void rc_ch_3_int_handler() {
  rc_update_ch_value(g_rc_ch_pins[2], &g_rc_ch_timer_begins[2], &g_rc_ch_values[2]);
}

void rc_ch_4_int_handler() {
  rc_update_ch_value(g_rc_ch_pins[3], &g_rc_ch_timer_begins[3], &g_rc_ch_values[3]);
}

}

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  Serial.begin(115200);
  
  const uint8_t kNChannel = 4;
  crim::rc_init(kNChannel);
  
  while (true) {
    for (uint8_t i=0; i<kNChannel; ++i) {
      Serial.println(crim::rc_read(i+1), DEC);
    }
    Serial.println("");
    
    delay(100);
  }
   
  Serial.end();
  return 0;
}
