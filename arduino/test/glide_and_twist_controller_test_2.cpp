#include <arduino-core/Arduino.h>
#include <arduino-core/wiring_private.h>// for voidFuncPtr
#include <actuator/servo.hpp>
#include <actuator/electronics_speed_controller.hpp>

class GlideAndTwistController {
 public:
  GlideAndTwistController(): int0_pin_(2),led_pin_(13) {
    pinMode(int0_pin_, INPUT_PULLUP);
    attachInterrupt(0, (voidFuncPtr) &GlideAndTwistController::int0_handler, FALLING);
    
    pinMode(led_pin_, OUTPUT);
  }
  
  ~GlideAndTwistController() {
  }
  
  bool led_state() {
    return led_state_;
  }
  
  int64_t counter() {
    return counter_;
  }
  
  uint8_t led_pin() {
    return led_pin_;
  }
  
  bool twist_dir() {
    return twist_dir_;
  }
  
 private:
  const uint8_t int0_pin_;
  const uint8_t led_pin_;
  static int64_t counter_;
  static bool led_state_;
  static bool twist_dir_;
  
  void int0_handler() {
    led_state_ = !led_state_;
    twist_dir_ = !twist_dir_;
    counter_++;
  }
};

bool GlideAndTwistController::led_state_ = LOW;
bool GlideAndTwistController::twist_dir_ = LOW;
int64_t GlideAndTwistController::counter_ = 0;

int main() {
  init();
  Serial.begin(9600);
  
  const uint8_t servo_pin = 9;
  
  Servo servo;
  servo.attach(servo_pin);
  servo.setSpeed(545.455);// degree per second; = 0.11s/60degree
  servo.write(0); 
  
  const uint8_t servo2_pin = 8;
  
  Servo servo2;
  servo2.attach(servo2_pin);
  servo2.setSpeed(545.455);// degree per second; = 0.11s/60degree
  servo2.write(0); 

  const uint8_t pwm_pin = 6;
  crim::ElectronicsSpeedController esc(pwm_pin);
  pinMode(pwm_pin, OUTPUT);

  const uint8_t servo3_pin = 7;
  
  Servo servo3;
  servo3.attach(servo3_pin);
  servo3.setSpeed(545.455);// degree per second; = 0.11s/60degree
  servo3.write(20); 
      
  GlideAndTwistController gtc;
  
  while (true) {
    digitalWrite(gtc.led_pin(), gtc.led_state());
    Serial.print("counter= ");
    Serial.println(static_cast<long int>(gtc.counter()));
    
    if(gtc.twist_dir()) {
      servo.write(30); 
      servo2.write(30);
    }
    else {
      servo.write(120);
      servo2.write(120);
    }

    //double duty_cycle = 90;
    //esc.set_duty_cycle(duty_cycle);
    
    servo3.write(90); 
    
    delay(100);
  }
  
  return 0;
}
