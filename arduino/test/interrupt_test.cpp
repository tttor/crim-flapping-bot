#include <arduino-core/Arduino.h>

int pin = 13;
volatile int state = LOW;
volatile int16_t counter = 0;

void blink()
{
  state = !state;
  counter++;
}

void setup()
{
  pinMode(pin, OUTPUT);
  attachInterrupt(0, blink, CHANGE);
  Serial.begin(9600);
}

void loop()
{
  digitalWrite(pin, state);
  Serial.println(static_cast<long int>(counter));
  delay(100);
}

int main() {
  init();
  setup();
  
  while (true) {
    loop();
  }
  
  return 0;
}
