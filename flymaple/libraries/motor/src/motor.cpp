#include <assert.h>
#include "wirish.h"
#include "motor/motor.h"

using namespace crim;

Motor Motor::motor;
const unsigned char Motor::pin[4] = {D28,D27,D11,D12};

Motor::Motor()
{
	/* pin mode setup related to motor should has to be put here*/
	pinMode(D28,PWM);
	pinMode(D27,PWM);
	pinMode(D11,PWM);
	pinMode(D12,PWM);
	Timer3.setPeriod(2080);
}

Motor::Motor(unsigned long frequency)
{
	/* pin mode setup related to motor should has to be put here*/
	pinMode(D28,PWM);
	pinMode(D27,PWM);
	pinMode(D11,PWM);
	pinMode(D12,PWM);
	setFrequency(frequency);
}

//set the PWM frequency in hertz maximum 1MHz
	unsigned long Motor::setFrequency(unsigned long frequency){
	uint32 period = (1000000/frequency);
	Timer3.setPeriod(period);
	return period;
}

//set the PWM duty cycle in % from 0-100
void Motor::setDuty(unsigned short pin, unsigned short duty_on){
	pwmWrite(pin, (Timer3.getOverflow()/100)*duty_on);
}

Motor::~Motor()
{
}

