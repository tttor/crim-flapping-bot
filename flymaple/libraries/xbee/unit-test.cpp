// Unit test: Xbee
// This sends back what is received

#include <wirish/wirish.h>

static const int BAUD_RATE = 9600;

void setup() {
    Serial1.begin(BAUD_RATE);
}

void loop() {
    while (Serial1.available()) {
        Serial1.write(Serial1.read());
    }
}

// Force init() to be called before anything else.
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
