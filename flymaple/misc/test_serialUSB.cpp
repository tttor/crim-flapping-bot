// Tests SerialUSB functionality.

#include <wirish/wirish.h>

void setup() {
    /* Set up the LED to blink  */
    pinMode(BOARD_LED_PIN, OUTPUT);
}

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
    setup();
    
    while (true) {
      SerialUSB.println("vektor"
                        "dewanto");
      SerialUSB.println("hi");      

      toggleLED();
      delay(1000);
    }
    
    return 0;
}
