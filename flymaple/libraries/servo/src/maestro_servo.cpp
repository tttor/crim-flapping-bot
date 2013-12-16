#include "servo/maestro_servo.h"

using namespace crim;

MaestroServo::MaestroServo(uint8_t addr, uint16_t min_pos, uint16_t max_pos)
    : addr_(addr), min_pos_(min_pos), max_pos_(max_pos) {
  Serial2.begin(9600);
}

MaestroServo::~MaestroServo() {
  Serial2.end();
}

void MaestroServo::go_to(uint16_t pos) {
  if (pos > max_pos_) 
    pos = max_pos_;
  else if (pos < min_pos_)
    pos = min_pos_;
  
  pos = (map(pos, 0, 180, 2400, 9600)); //Map the pos angle to the corresponding PWM pulse. range : 600-2400 uS
  
  byte serialBytes[6]; //Create the byte array object that will hold the communication packet.
  serialBytes[0] = 0xAA; // Command byte: Set pos.
  serialBytes[1] = 0x0C;
  serialBytes[2] = 0x04;
  serialBytes[3] = addr_; // First byte holds channel number.
  serialBytes[4] = pos & 0x7F; // Second byte holds the lower 7 bits of pos.
  serialBytes[5] = (pos >> 7) & 0x7F; // Third byte holds the bits 7-13 of pos.
  
  Serial2.write(serialBytes, sizeof(serialBytes)); //Write the byte array to the serial port.
}

uint8_t MaestroServo::get_addr() {
  return addr_;
}

void MaestroServo::goto_max() {
  go_to(max_pos_);
}

void MaestroServo::goto_min() {
  go_to(min_pos_);
}

void MaestroServo::goto_star(uint16_t idx) {
  go_to(star_poses.at(idx));
}
