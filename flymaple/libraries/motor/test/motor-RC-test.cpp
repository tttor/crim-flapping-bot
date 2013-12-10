/**
 * @file   main.cpp
 * @author breadbread1984 <breadbread1984@163.com>
 * @date   Sat Jul 21 15:12:00 2012
 * 
 * @section DESCRIPTION 
 * 
 * openDrone Quadcopter Main function File
 * 
 * @section LICENSE
 * 
 * GPLv3 
 */


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

unsigned long period;
unsigned long chan1PPM_init;
unsigned long chan2PPM_init;

/* PWM pin for left tire */
#define PWM_PIN_1 		D28

/* PWM pin for right tire */
#define PWM_PIN_2		D27

/* Pin for RC controller input */
#define	Channel1Pin		D31
#define Channel2Pin		D32

/* Pin for direction control */
#define dirLeft1		D35
#define dirLeft2		D36
#define dirRight1		D37
#define dirRight2		D26


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


/* Force init() to be called before anything else. */
__attribute__((constructor)) void premain() {
    init();
}

/* drive the robot forward with given PWM value for the left and right tire */
void forward(uint8 left_PWM, uint8 right_PWM){
	digitalWrite(dirLeft1, HIGH);
	digitalWrite(dirLeft2, LOW);
	digitalWrite(dirRight1, HIGH);
	digitalWrite(dirRight2, LOW);
	Motor::setDuty(PWM_PIN_1, left_PWM);
	Motor::setDuty(PWM_PIN_2, right_PWM);
}

/* drive the robot backward with given PWM value for the left and right tire */
void backward(uint8 left_PWM, uint8 right_PWM){
	digitalWrite(dirLeft1, LOW);
	digitalWrite(dirLeft2, HIGH);
	digitalWrite(dirRight1, LOW);
	digitalWrite(dirRight2, HIGH);
	Motor::setDuty(PWM_PIN_1, left_PWM);
	Motor::setDuty(PWM_PIN_2, right_PWM);
}

/* hard stop the motor */
void stop(){
	digitalWrite(dirLeft1, HIGH);
	digitalWrite(dirLeft2, HIGH);
}

void setup()
{
    /* Set up the direction control pin  */
    pinMode(dirLeft1, OUTPUT);
    pinMode(dirLeft2, OUTPUT);
    pinMode(dirRight1, OUTPUT);
    pinMode(dirRight2, OUTPUT);

    /* Turn on PWM on pin PWM_PIN */
    pinMode(PWM_PIN_1, PWM);
    pinMode(PWM_PIN_2, PWM);
    
    /* attach RC receiver to interrupt */
    pinMode(Channel1Pin, INPUT);
	attachInterrupt(Channel1Pin, handler_CH1, CHANGE);
	
	/* attach RC receiver to interrupt */
	pinMode(Channel2Pin, INPUT);
	attachInterrupt(Channel2Pin, handler_CH2, CHANGE);
	
	period = Motor::setFrequency(50000);
	
    /* Send a message out USART2  */
    Serial2.begin(9600);
}

void waitForController(){
	while(chan1PPM < 1500 || chan1PPM > 1550);
	while(chan2PPM < 1500 || chan2PPM > 1550);
	chan1PPM_init = chan1PPM;
	chan2PPM_init = chan2PPM;
}

void loop(){
	SerialUSB.print("\n\rCh 1 : ");
	SerialUSB.print(chan1PPM, DEC);
	SerialUSB.print("\n\rCh 2 : ");
	SerialUSB.print(chan2PPM, DEC);
	SerialUSB.print("\n\rPeriod value :  ");
	SerialUSB.print(period);
	SerialUSB.print("\n\rPrescale Factor timer 3 : ");
	SerialUSB.print(Timer3.getPrescaleFactor(), 16);
	SerialUSB.print("\n\rOverflow timer 3 : ");
	SerialUSB.print(Timer3.getOverflow(), 16);
	
	if(chan2PPM > chan2PPM_init){
		// drive to the the front right
		if(chan1PPM > chan1PPM_init){
			forward((chan2PPM-chan2PPM_init+(chan1PPM-chan1PPM_init))/10, (chan2PPM-chan2PPM_init)/10);
		}
		else{
			forward((chan2PPM-chan2PPM_init)/10, (chan2PPM-chan2PPM_init+(chan1PPM_init-chan1PPM))/10);
		}
	}
	else{
		if(chan1PPM > chan1PPM_init){
			backward((chan2PPM_init-chan2PPM+(chan1PPM-chan1PPM_init))/10, (chan2PPM_init-chan2PPM)/10);
		}
		else{
			backward((chan2PPM_init-chan2PPM)/10, (chan2PPM_init-chan2PPM+(chan1PPM_init-chan1PPM))/10);
		}
	}
	
/*	Motor::setDuty(PWM_PIN, PWM_VAL);
	if(PWM_VAL < 80)
		PWM_VAL+=1;
	else
		PWM_VAL = 10;
	SerialUSB.print("\n\rPWM value now : ");
	SerialUSB.print(PWM_VAL); */
	
	delay(50);
}

/* Please Do Not Remove & Edit Following Code */
int main(void) {
    setup();
    
    waitForController();

    while (true) {
        loop();
    }

    return 0;
}
