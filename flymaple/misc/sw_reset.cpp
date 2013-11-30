#include<nvic.h>

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
    while (true) {
        nvic_sys_reset();
        delay(1000);
    }
    return 0;
}
