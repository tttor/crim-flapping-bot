#include "wirish.h"
#include "maestro_servo/maestro_servo.h"

void setup() {
    maestroInit();
  
}

void loop() {
    maestroSetServo(0, 0);
    maestroSetServo(1, 0);
    delay(1000);

    maestroSetServo(0, 90);
    maestroSetServo(1, 90);
    delay(1000);
    
    maestroSetServo(0, 180);
    maestroSetServo(1, 180);
    delay(1000);

    maestroSetServo(0, 90);
    maestroSetServo(1, 90);
    delay(1000);
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
