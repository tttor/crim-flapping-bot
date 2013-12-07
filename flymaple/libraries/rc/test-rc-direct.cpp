// Unit test: RC
// This sends back what is received

#include <wirish/wirish.h>

volatile uint16_t chan1PPM;
volatile uint16_t chan1begin;
volatile uint16_t chan1end;

volatile uint16_t chan2PPM;
volatile uint16_t chan2begin;
volatile uint16_t chan2end;

volatile uint16_t chan3PPM;
volatile uint16_t chan3begin;
volatile uint16_t chan3end;

volatile uint16_t chan4PPM;
volatile uint16_t chan4begin;
volatile uint16_t chan4end;

volatile uint16_t chan5PPM;
volatile uint16_t chan5begin;
volatile uint16_t chan5end;

volatile uint16_t chan6PPM;
volatile uint16_t chan6begin;
volatile uint16_t chan6end;

volatile uint16_t chan7PPM;
volatile uint16_t chan7begin;
volatile uint16_t chan7end;

volatile uint16_t chan8PPM;
volatile uint16_t chan8begin;
volatile uint16_t chan8end;

void handler_CH1(void);
void handler_CH2(void);
void handler_CH3(void);
void handler_CH4(void);
void handler_CH5(void);
void handler_CH6(void);
void handler_CH7(void);
void handler_CH8(void);

#define	Channel1Pin		D31
#define Channel2Pin		D32
#define Channel3Pin		D33
#define Channel4Pin		D34
#define Channel5Pin		D35
#define Channel6Pin		D36
#define Channel7Pin		D37
#define Channel8Pin		D26

void setup() {
	
	pinMode(Channel1Pin, INPUT);
	attachInterrupt(Channel1Pin, handler_CH1, CHANGE);
	
	pinMode(Channel2Pin, INPUT);
	attachInterrupt(Channel2Pin, handler_CH2, CHANGE);
	
	pinMode(Channel3Pin, INPUT);
	attachInterrupt(Channel3Pin, handler_CH3, CHANGE);
	
	pinMode(Channel4Pin, INPUT);
	attachInterrupt(Channel4Pin, handler_CH4, CHANGE);
	
	pinMode(Channel5Pin, INPUT);
	attachInterrupt(Channel5Pin, handler_CH5, CHANGE);
	
	pinMode(Channel6Pin, INPUT);
	attachInterrupt(Channel6Pin, handler_CH6, CHANGE);
	
	pinMode(Channel7Pin, INPUT);
	attachInterrupt(Channel7Pin, handler_CH7, CHANGE);
	
	pinMode(Channel8Pin, INPUT);
	attachInterrupt(Channel8Pin, handler_CH8, CHANGE);
	
}

void handler_CH1(void) {
	
	uint16_t total = 0;
	
	if(digitalRead(Channel1Pin) == 1) {
		
		chan1begin = micros();
	} else {
		
		if(chan1begin != 0) {
			chan1end = micros();
			total = chan1end-chan1begin;
			if((total > 950) && (total < 2000)) {
				chan1PPM = total;
			}
			chan1begin = 0;
		}
	}
}

void handler_CH2(void) {
	
	uint16_t total = 0;
	
	if(digitalRead(Channel2Pin) == 1) {
		
		chan2begin = micros();
	} else {
		
		if(chan2begin != 0) {
			chan2end = micros();
			total = chan2end - chan2begin;
			if((total > 950) && (total < 2000)) {
				chan2PPM = total;
			}
			chan2begin = 0;
		}
	}
}

void handler_CH3(void) {
	
	uint16_t total = 0;
	
	if(digitalRead(Channel3Pin) == 1) {
		
		chan3begin = micros();
	} else {
		
		if(chan3begin != 0) {
			chan3end = micros();
			total = chan3end - chan3begin;
			if((total > 950) && (total < 2000)) {
				chan3PPM = total;
			}
			chan3begin = 0;
		}
	}
}

void handler_CH4(void) {
	
	uint16_t total = 0;
	
	if(digitalRead(Channel4Pin) == 1) {
		
		chan4begin = micros();
	} else {
		
		if(chan4begin != 0) {
			chan4end = micros();
			total = chan4end - chan4begin;
			if((total > 950) && (total < 2000)) {
				chan4PPM = total;
			}
			chan4begin = 0;
		}
	}
}

void handler_CH5(void) {
	
	uint16_t total = 0;
	
	if(digitalRead(Channel5Pin) == 1) {
		
		chan5begin = micros();
	} else {
		
		if(chan5begin != 0) {
			chan5end = micros();
			total = chan5end - chan5begin;
			if((total > 950) && (total < 2000)) {
				chan5PPM = total;
			}
			chan5begin = 0;
		}
	}
}

void handler_CH6(void) {
	
	uint16_t total = 0;
	
	if(digitalRead(Channel6Pin) == 1) {
		
		chan6begin = micros();
		
	} else {
		
		if(chan6begin != 0) {
			chan6end = micros();
			total = chan6end - chan6begin;
			if((total > 950) && (total < 2000)) {
				chan6PPM = total;
			}
			chan6begin = 0;
		}
	}
}

void handler_CH7(void) {
	
	uint16_t total = 0;
	
	if(digitalRead(Channel7Pin) == 1) {
		
		chan7begin = micros();
	} else {
		
		if(chan7begin != 0) {
			chan7end = micros();
			total = chan7end - chan7begin;
			if((total > 950) && (total < 2000)) {
				chan7PPM = total;
			}
			chan7begin = 0;
		}
	}
}

void handler_CH8(void) {
	
	uint16_t total = 0;
	
	if(digitalRead(Channel8Pin) == 1) {
		
		chan8begin = micros();
	} else {
		
		if(chan8begin != 0) {
			chan8end = micros();
			total = chan8end - chan8begin;
			if((total > 950) && (total < 2000)) {
				chan8PPM = total;
			}
			chan8begin = 0;
		}
	}
}


// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
	
	setup();
   
    while (true) {
		SerialUSB.print("\n\rCh 1 : ");
		SerialUSB.print(chan1PPM, DEC);
		SerialUSB.print("\n\rCh 2 : ");
		SerialUSB.print(chan2PPM, DEC);
		SerialUSB.print("\n\rCh 3 : ");
		SerialUSB.print(chan3PPM, DEC);
		SerialUSB.print("\n\rCh 4 : ");
		SerialUSB.print(chan4PPM, DEC);
		SerialUSB.print("\n\rCh 5 : ");
		SerialUSB.print(chan5PPM, DEC);
		SerialUSB.print("\n\rCh 6 : ");
		SerialUSB.print(chan6PPM, DEC);
		SerialUSB.print("\n\rCh 7 : ");
		SerialUSB.print(chan7PPM, DEC);
		SerialUSB.print("\n\rCh 8 : ");
		SerialUSB.print(chan8PPM, DEC);
		SerialUSB.print("\n\n\n\r");
		
		delay(200);
    }
    return 0;
}
