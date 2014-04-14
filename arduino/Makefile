# REF: http://www.ashleymills.com/node/327
# For board-specific setting see: https://github.com/arduino/Arduino/blob/master/hardware/arduino/boards.txt
MCU= -mmcu=atmega328p # atmega328p, atmega2560
MCU_TAG= m328p # See the avrdude manual: m328p, m2560
CPU_SPEED= -DF_CPU=16000000UL
UPLOAD_BAUDRATE= 57600 # 115200 
UPLOAD_PROTOCOL= arduino # arduino, wiring, avr109

CXX_COMPILER= avr-gcc
CFLAGS= $(MCU) $(CPU_SPEED) -Os -w -Wl,--gc-sections -ffunction-sections -fdata-sections
PORT= /dev/ttyUSB0 # /dev/ttyACM0
BUILD_DIR= ./build

INCLUDE= -I ./include/
INCLUDE_2= -I ./include/arduino-core # Because some files from arduino-core include headers using #include <new.h> instead of #include <arduino/new>
INCLUDE_3= -I ../common/mavlink/include
INCLUDE_4= -I ../common/macro/include
LIBS= -L ./lib -larduino-core -lm # _must_ be in this order, see: http://stackoverflow.com/questions/11111966/eclipse-arduino-linker-trouble

# Put objects to build here
# the src files is included here as a workaround for problem: fail to link arduino core when building the corresponding lib
OBJECTS= main.cpp
#OBJECTS= main.cpp \
				#./src/sensor/two_phase_incremental_encoder.cpp \
				#./src/sensor/chr_um6_packet.cpp \
				#./src/sensor/chr_um6.cpp \
				#./src/comm/arduino_mavlink_packet_handler.cpp \
				#./src/actuator/electronics_speed_controller.cpp \
				#./src/actuator/servo.cpp
#OBJECTS= main.cpp \
				#./src/actuator/electronics_speed_controller.cpp \
				#./src/actuator/servo.cpp
				
default:
	mkdir -p $(BUILD_DIR)
	$(CXX_COMPILER) $(CFLAGS) $(INCLUDE) $(INCLUDE_2) $(INCLUDE_3) $(INCLUDE_4) $^ -o $(BUILD_DIR)/main.elf $(OBJECTS) $(LIBS)
	avr-objcopy -O ihex $(BUILD_DIR)/main.elf $(BUILD_DIR)/main.hex

upload:
	avrdude -V -F -D -p $(MCU_TAG) -c $(UPLOAD_PROTOCOL) -b $(UPLOAD_BAUDRATE) -Uflash:w:$(BUILD_DIR)/main.hex -P$(PORT)

install: upload

clean:
	$(shell rm *.elf 2> /dev/null)
	$(shell rm *.hex 2> /dev/null)
	$(shell rm *.o 2> /dev/null)
	rm -rf $(BUILD_DIR)

