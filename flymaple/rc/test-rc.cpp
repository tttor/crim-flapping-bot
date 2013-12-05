// Unit test: RC using library rc_capture.h
// This sends back what is received

#include <wirish/wirish.h>
#include <rc/rc_capture.h>

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
	init();
}

int main(void) {
  
	using namespace crim;
	
  RC_Capture RC;
  
  RC.setupRC();
   
	while (true) {
		SerialUSB.print("\n\rCh 1 : ");
		SerialUSB.print(RC.readRC(1), DEC);
    SerialUSB.print("\n\rCh 1 : ");
		SerialUSB.print(RC.readRC(2), DEC);
    SerialUSB.print("\n\rCh 1 : ");
		SerialUSB.print(RC.readRC(3), DEC);
    SerialUSB.print("\n\rCh 1 : ");
		SerialUSB.print(RC.readRC(4), DEC);
    SerialUSB.print("\n\rCh 1 : ");
		SerialUSB.print(RC.readRC(5), DEC);
    SerialUSB.print("\n\rCh 1 : ");
		SerialUSB.print(RC.readRC(6), DEC);
    SerialUSB.print("\n\rCh 1 : ");
		SerialUSB.print(RC.readRC(7), DEC);
    SerialUSB.print("\n\rCh 1 : ");
		SerialUSB.print(RC.readRC(8), DEC);
		SerialUSB.print("\n\n\r");
		
		delay(200);
	}
	return 0;
}
