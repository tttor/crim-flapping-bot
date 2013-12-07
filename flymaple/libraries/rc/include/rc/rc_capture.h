// @author saripudin
#ifndef RC_CAPTURE_H
#define RC_CAPUTRE_H

#include "wirish.h"

#define	Channel1Pin		D31
#define Channel2Pin		D32
#define Channel3Pin		D33
#define Channel4Pin		D34
#define Channel5Pin		D35
#define Channel6Pin		D36
#define Channel7Pin		D37
#define Channel8Pin		D26

namespace crim{

volatile uint16 chanPPM[8];
volatile uint16 chanBegin[8];
volatile uint16 chanEnd[8];

void handler_CH1(void);
void handler_CH2(void);
void handler_CH3(void);
void handler_CH4(void);
void handler_CH5(void);
void handler_CH6(void);
void handler_CH7(void);
void handler_CH8(void);

class RC_Capture {
 public:
 
  /**
    @brief
  */
  RC_Capture();
  
  /**
    @brief
  */
  uint16 readRC(uint8 channel);

  /**
    @brief
  */
  void setupRC(void);

 private:

};

void RC_Capture::setupRC(void) {
	
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

RC_Capture::RC_Capture() {
  
}

uint16 RC_Capture::readRC(uint8 channel)
{
	return chanPPM[channel - 1];
}

void handler_CH1(void) {
	
	uint16 total = 0;
	
	if(digitalRead(Channel1Pin) == 1) {
		
		chanBegin[0] = micros();
	} else {
		
		if(chanBegin[0] != 0) {
			chanEnd[0] = micros();
			total = chanEnd[0] - chanBegin[0];
			if((total > 950) && (total < 2000)) {
				chanPPM[0] = total;
			}
			chanBegin[0] = 0;
		}
	}
}

void handler_CH2(void) {
	
	uint16 total = 0;
	
	if(digitalRead(Channel2Pin) == 1) {
		
		chanBegin[1] = micros();
	} else {
		
		if(chanBegin[1] != 0) {
			chanEnd[1] = micros();
			total = chanEnd[1] - chanBegin[1];
			if((total > 950) && (total < 2000)) {
				chanPPM[1] = total;
			}
			chanBegin[1] = 0;
		}
	}
}

void handler_CH3(void) {
	
	uint16 total = 0;
	
	if(digitalRead(Channel3Pin) == 1) {
		
		chanBegin[2] = micros();
	} else {
		
		if(chanBegin[2] != 0) {
			chanEnd[2] = micros();
			total = chanEnd[2] - chanBegin[2];
			if((total > 950) && (total < 2000)) {
				chanPPM[2] = total;
			}
			chanBegin[2] = 0;
		}
	}
}

void handler_CH4(void) {
	
	uint16 total = 0;
	
	if(digitalRead(Channel4Pin) == 1) {
		
		chanBegin[3] = micros();
	} else {
		
		if(chanBegin[3] != 0) {
			chanEnd[3] = micros();
			total = chanEnd[3] - chanBegin[3];
			if((total > 950) && (total < 2000)) {
				chanPPM[3] = total;
			}
			chanBegin[3] = 0;
		}
	}
}

void handler_CH5(void) {
	
	uint16 total = 0;
	
	if(digitalRead(Channel5Pin) == 1) {
		
		chanBegin[4] = micros();
	} else {
		
		if(chanBegin[4] != 0) {
			chanEnd[4] = micros();
			total = chanEnd[4] - chanBegin[4];
			if((total > 950) && (total < 2000)) {
				chanPPM[4] = total;
			}
			chanBegin[4] = 0;
		}
	}
}

void handler_CH6(void) {
	
	uint16 total = 0;
	
	if(digitalRead(Channel6Pin) == 1) {
		
		chanBegin[5] = micros();
	} else {
		
		if(chanBegin[5] != 0) {
			chanEnd[5] = micros();
			total = chanEnd[5] - chanBegin[5];
			if((total > 950) && (total < 2000)) {
				chanPPM[5] = total;
			}
			chanBegin[5] = 0;
		}
	}
}

void handler_CH7(void) {
	
	uint16 total = 0;
	
	if(digitalRead(Channel7Pin) == 1) {
		
		chanBegin[6] = micros();
	} else {
		
		if(chanBegin[6] != 0) {
			chanEnd[6] = micros();
			total = chanEnd[6] - chanBegin[6];
			if((total > 950) && (total < 2000)) {
				chanPPM[6] = total;
			}
			chanBegin[6] = 0;
		}
	}
}

void handler_CH8(void) {
	
	uint16 total = 0;
	
	if(digitalRead(Channel8Pin) == 1) {
		
		chanBegin[7] = micros();
	} else {
		
		if(chanBegin[7] != 0) {
			chanEnd[7] = micros();
			total = chanEnd[7] - chanBegin[7];
			if((total > 950) && (total < 2000)) {
				chanPPM[7] = total;
			}
			chanBegin[7] = 0;
		}
	}
}

}// namespace crim
#endif
