// Blinks the built-in LED
#include "wirish/wirish.h"

void setup() {
    pinMode(BOARD_LED_PIN, OUTPUT);
}

void loop() {
    SerialUSB.println("looping...");

    togglePin(BOARD_LED_PIN);
    delay(1000);
}

__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
    setup();

    while (true) {
        loop();
    }
    return 0;
}
