#ifndef MOTOR_H
#define MOTOR_H

namespace crim {

class Motor {
	static Motor motor;
	static const unsigned char pin[4];
  
	Motor();
	
	
public:
	Motor(unsigned long frequency);
	~Motor();
	static unsigned long setFrequency(unsigned long frequency);
	static void setDuty(unsigned short pin, unsigned short duty_on);
};

}
#endif
