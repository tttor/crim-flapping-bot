# REF: http://www.ashleymills.com/node/327
BOARD=arduino
MCU=-mmcu=atmega328p 
CPU_SPEED=-DF_CPU=16000000UL

CC=avr-gcc
CXX=avr-g++
INCLUDE=-I .
CFLAGS=$(MCU) $(CPU_SPEED) -Os -w
LIBNAME=lib$(BOARD)-core.a
INCDIR=../../include/$(BOARD)-core
LIBDIR=../../lib

OBJECTS=  pins_arduino.o wiring.o wiring_analog.o wiring_digital.o \
					wiring_pulse.o wiring_shift.o HardwareSerial.o Print.o   \
		 			Tone.o WMath.o WString.o WInterrupts.o new.o

default: $(OBJECTS)
	avr-ar rcs $(LIBNAME) $^
	mkdir -p $(INCDIR)
	mkdir -p $(LIBDIR)
	cp *.h $(INCDIR)/
	mv $(LIBNAME) $(LIBDIR)
	rm *.o

%.o : %.c
	$(CC) $< $(CFLAGS) $(INCLUDE) -c -o $@

%.o : %.cpp
	$(CXX) $< $(CFLAGS) $(INCLUDE) -c -o $@
	
clean:
	$(shell rm $(LIBDIR)/*.a 2> /dev/null)
