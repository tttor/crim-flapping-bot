#include "wirish.h"

#define LED_RATE 500000    // in microseconds; should give 0.5Hz toggles

HardwareTimer timer(2);

void handler_led(void) {
    toggleLED();
    //SerialUSB.println("Bamm");// see: http://forums.leaflabs.com/topic.php?id=139
}

void setup() {
    // Set up the LED to blink
    pinMode(D13, OUTPUT);

    // Pause the timer while we're configuring it
    timer.pause();

    // Set up period
    timer.setPeriod(LED_RATE); // in microseconds

    // Set up an interrupt on channel 1
    timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
    timer.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
    timer.attachCompare1Interrupt(handler_led);

    // Refresh the timer's count, prescale, and overflow
    timer.refresh();

    // Start the timer counting
    timer.resume();
}

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {
    init();
}

int main() {
  setup();

  while(true) {
    SerialUSB.println("looping");
    delay(1000);
  }

  return 0;
}
