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

//namespace crim{

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
//}// namespace crim
#endif
