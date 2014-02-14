# REF: http://www.ashleymills.com/node/327
# For board-specific setting see: https://github.com/arduino/Arduino/blob/master/hardware/arduino/boards.txt
MCU= -mmcu=atmega2560 # atmega328p
MCU_TAG= m2560 # See the avrdude manual: m328p
CPU_SPEED= -DF_CPU=16000000UL
UPLOAD_BAUDRATE= 115200 
UPLOAD_PROTOCOL= wiring # arduino, wiring, avr109

CXX_COMPILER= avr-gcc
CFLAGS= $(MCU) $(CPU_SPEED) -Os -w -Wl,--gc-sections -ffunction-sections -fdata-sections
PORT= /dev/ttyACM0
BUILD_DIR= ./build

INCLUDE= -I ./include/
INCLUDE_2= -I ./include/arduino-core # Because some files from arduino-core include headers using #include <new.h> instead of #include <arduino/new>
LIBS= -L ./lib -larduino-core -lm # _must_ be in this order, see: http://stackoverflow.com/questions/11111966/eclipse-arduino-linker-trouble

# Put objects to build here
# the encoder src files is included here as a workaround for problem: fail to link arduino core when building the encoder lib
#OBJECTS= main.cpp
OBJECTS= main.cpp ./src/encoder/two_phase_incremental_encoder.cpp ./src/sensor/chr_um6_packet.cpp ./src/sensor/chr_um6.cpp

default:
	mkdir -p $(BUILD_DIR)
	$(CXX_COMPILER) $(CFLAGS) $(INCLUDE) $(INCLUDE_2) $^ -o $(BUILD_DIR)/main.elf $(OBJECTS) $(LIBS)
	avr-objcopy -O ihex $(BUILD_DIR)/main.elf $(BUILD_DIR)/main.hex

upload:
	avrdude -V -F -D -p $(MCU_TAG) -c $(UPLOAD_PROTOCOL) -b $(UPLOAD_BAUDRATE) -Uflash:w:$(BUILD_DIR)/main.hex -P$(PORT)

install: upload

clean:
	$(shell rm *.elf 2> /dev/null)
	$(shell rm *.hex 2> /dev/null)
	$(shell rm *.o 2> /dev/null)
	rm -rf $(BUILD_DIR)

