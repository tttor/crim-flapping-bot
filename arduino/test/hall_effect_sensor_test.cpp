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
  Serial.begin(9600);
  
  int int0_pin = 2;
  pinMode(int0_pin, INPUT_PULLUP);
  attachInterrupt(0, blink, CHANGE);
}

void loop()
{
  digitalWrite(pin, state);
  Serial.print("counter= ");
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
