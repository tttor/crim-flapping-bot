#include "wirish.h"
#include "motor/motor.h"

using namespace crim;

volatile uint16 chan1PPM;
volatile uint16 chan1begin;
volatile uint16 chan1end;

volatile uint16 chan2PPM;
volatile uint16 chan2begin;
volatile uint16 chan2end;

void handler_CH1(void);
void handler_CH2(void);

unsigned long chan1PPM_init;
unsigned long chan2PPM_init;

/* Pin for RC controller input */
#define	Channel1Pin		D31
#define Channel2Pin		D32

void handler_CH1(void) {
	
	uint16 total = 0;
	
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
	
	uint16 total = 0;
	
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

void waitForController() {
  while(chan1PPM < 1500 || chan1PPM > 1550) continue;
  while(chan2PPM < 1500 || chan2PPM > 1550) continue;
  
  chan1PPM_init = chan1PPM;
  chan2PPM_init = chan2PPM;
}

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
  // Set up motors
  unsigned char r_motor_pwm_pin = D12;
  unsigned char r_motor_dir_1_pin = D24;
  unsigned char r_motor_dir_2_pin = D14;
  unsigned long r_frequency = 50000;

  crim::Motor right_motor(r_motor_pwm_pin, r_motor_dir_1_pin, r_motor_dir_2_pin, r_frequency);
  
  unsigned char l_motor_pwm_pin = D28;
  unsigned char l_motor_dir_1_pin = D27;
  unsigned char l_motor_dir_2_pin = D11;
  unsigned long l_frequency = r_frequency;
  
  crim::Motor left_motor(l_motor_pwm_pin, l_motor_dir_1_pin, l_motor_dir_2_pin, l_frequency);
  
  // Set up RC
  pinMode(Channel1Pin, INPUT);
  attachInterrupt(Channel1Pin, handler_CH1, CHANGE);

  pinMode(Channel2Pin, INPUT);
  attachInterrupt(Channel2Pin, handler_CH2, CHANGE);
  
  waitForController();
  
  // The loop    
  while (true) {
    if (chan2PPM > chan2PPM_init) {
      if (chan1PPM > chan1PPM_init) {
        left_motor.forward((chan2PPM-chan2PPM_init+(chan1PPM-chan1PPM_init))/10);
        right_motor.forward((chan2PPM-chan2PPM_init)/10);
      } else {
        left_motor.forward((chan2PPM-chan2PPM_init)/10);
        right_motor.forward((chan2PPM-chan2PPM_init+(chan1PPM_init-chan1PPM))/10);
      }
    } else {
      if(chan1PPM > chan1PPM_init){
        left_motor.backward((chan2PPM_init-chan2PPM+(chan1PPM-chan1PPM_init))/10);
        right_motor.backward((chan2PPM_init-chan2PPM)/10);
      } else{
        left_motor.backward((chan2PPM_init-chan2PPM)/10);
        right_motor.backward((chan2PPM_init-chan2PPM+(chan1PPM_init-chan1PPM))/10);
      }
    }
    
    delay(50);
  }

  return 0;
}
