#include "wirish.h"
#include "servo/maestro_servo.h"

using namespace crim;

// Force init() to be called before anything else.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
  //////////////////////////////////////////////////////////////////////////////
  MaestroServo servo_1(0, 70, 180);
  servo_1.star_poses.push_back(90);
  
  while (true) {
    servo_1.goto_max();// fully upward
    delay(2000);
    
    servo_1.goto_star(0);// straight to the front
    delay(2000);
    
    servo_1.goto_min();//  just-right downward
    delay(2000);
  }    
  
  //////////////////////////////////////////////////////////////////////////////
  //MaestroServo servo_2(1, 90, 180);
  
  //while (true) {
    //servo_2.goto_min();
    //delay(2000);
    
    //servo_2.goto_max();
    //delay(2000);
  //}
  
  //////////////////////////////////////////////////////////////////////////////
  return 0;
}
