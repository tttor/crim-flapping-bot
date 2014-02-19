#include <arduino-core/Arduino.h>
#include <arduino-core/wiring_private.h>// for voidFuncPtr
#include <sensor/two_phase_incremental_encoder.hpp>

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  Serial.begin(115200);
  
  const size_t encoder_out_a_pin = 2;
  const size_t encoder_out_b_pin = 3;
  const uint64_t encoder_resolution = 360;
  crim::TwoPhaseIncrementalEncoder encoder(encoder_out_a_pin, encoder_out_b_pin, encoder_resolution);
  
  while (true) {
    Serial.println("===");
    Serial.println(static_cast<long int>(encoder.pos()));
    Serial.println(encoder.rot());
    delay(100);
  }
  
  Serial.end();
  return 0;
}
