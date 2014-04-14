#include <arduino-core/Arduino.h>
#include <arduino-core/wiring_private.h>// for voidFuncPtr
#include <actuator/servo.hpp>
#include <actuator/electronics_speed_controller.hpp>
#include <comm/one_channel_radio_control.hpp>


enum GearState{
    unknown,
    hi_pos,
    lo_pos,
    aux_pos,
    n_gear_state
 };
   
/**
 * (1) The hall-effect sensor can not be powered up using USB's power line
 * (2) To determine the gear state, we have to do an initialization in which the use throtle-up up to some speed then throtle-down, the controller will make a stop exactly at the beginning of hi_pos (using the variable named just_at_hi_pos)
 * (3) Use timer to determine the speed of the gear for prestop_speed
 * (4) Use 2 hall-effect sensors and 2 magnets, forming 2 pairs to indicate hi-pos and lo-pos. Those two sensors are connected to 2 external interrupt. For reading the ch3 pwm/throtle, we use PinChangeINTerrupt(PCINT), see http://rcarduino.blogspot.com/2012/03/need-more-interrupts-to-read-more.html
 * (5) Separate the two systems: glide and twist controller. For the former, use the SGDrone's glide controller. For the latter, build with arduinoi pro mini with 2 magnets different poles facing the sensor, as on (4). Put the magnet for each system in different radius. Tis 
 * (6) Put the magnet and sensors in the slowest rotating gear in order to ensure accuracy.
 */
class GlideAndTwistController {
 public:
  GlideAndTwistController(): interrupt_number_(1), interrupt_pin_(3),led_pin_(13) {
    pinMode(interrupt_pin_, INPUT_PULLUP);
    attachInterrupt(interrupt_number_, (voidFuncPtr) &GlideAndTwistController::interrupt_handler, FALLING);
    
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
  
  GearState state() {
    return state_;
  }
  
  bool just_at_hi_pos() {
    return just_at_hi_pos_;
  }
  
  void reset_just_at_hi_pos() {
    just_at_hi_pos_ = false;
  }
  
 private:
  const uint8_t interrupt_number_;
  const uint8_t interrupt_pin_;
  const uint8_t led_pin_;
  static int64_t counter_;
  static bool led_state_;
  static GearState state_; // cyclic: 0-> unknown, 1-> hi pos, 2-> lo pos, 3-> aux pos
  static bool just_at_hi_pos_;
  
  void interrupt_handler() {
    // Clycling the state
    if (state_==hi_pos) {
      state_ = lo_pos;
    }
    else if (state_==lo_pos) {
      state_ = aux_pos;
    }
    else if (state_==aux_pos) {
      state_ = hi_pos;
    }
    
    //
    counter_++;
    just_at_hi_pos_ = true;
    led_state_ = !led_state_;
  }
};

bool GlideAndTwistController::led_state_ = LOW;
int64_t GlideAndTwistController::counter_ = 0;
GearState GlideAndTwistController::state_ = unknown;
bool GlideAndTwistController::just_at_hi_pos_ = false;

int main() {
  init();
  Serial.begin(9600);
  
  // Setup servos for wing twists
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

  // Setup the ESC
  const uint8_t esc_pin = 7;
  const uint16_t min_throtle = 1052;
  const uint16_t max_throtle = 1896;
  const uint16_t arming_throtle = min_throtle;
  const uint16_t prestop_throtle = 1250;// TODO @tttor; this should be some speed limit, not the throtle limit. Otherwise this depends on the batt volt. level.
    
  Servo esc;
  esc.attach(esc_pin, min_throtle, max_throtle);
  esc.writeMicroseconds(arming_throtle);
  delay(2000);
  
  // 
  crim::rc_init();
  uint8_t throtle_gradient = 0;
  uint16_t past_throtle = min_throtle;
  uint16_t tol = 15;
  
  //
  GlideAndTwistController gtc;
  
  //gtc.init();
  //gtc.run_forever();
  
  while (true) {
    //
    digitalWrite(gtc.led_pin(), gtc.led_state());
    Serial.print("counter= "); Serial.println(static_cast<long int>(gtc.counter()));
    
    // Wing twist control
    if(gtc.state()==hi_pos) {
      servo.write(30); 
      servo2.write(30);
    }
    else if (gtc.state()==lo_pos) {
      servo.write(120);
      servo2.write(120);
    }
    
    // Flap control, gliding control
    uint16_t throtle = crim::rc_read();
    Serial.print("throtle= "); Serial.println(throtle, DEC);
    Serial.print("past throtle= "); Serial.println(past_throtle, DEC);
    
    if (throtle < min_throtle) {
      Serial.println("invalid throtle value!");
      continue;
    }
    
    //
    if ( (throtle >= (past_throtle-tol)) and (throtle <= (past_throtle+tol)) ) {
      throtle_gradient = 0;// no change
    }
    else if (throtle > (past_throtle+tol)) {
      throtle_gradient = 1;// up
      Serial.println("++++++++++++++++++++++++++++++");
    }
    else if (throtle < (past_throtle-tol)) {
      throtle_gradient = 2;// down
      Serial.println("DOWN DOWN DOWN DOWN DOWN DOWN ");
    } 
    past_throtle = throtle;
    
    if ((throtle < prestop_throtle)and(throtle_gradient==2)) {
      // Assume we are not at the hi pos
      gtc.reset_just_at_hi_pos();
      
      // maintain the speed till we get the right position to brake
      Serial.println("stanby to stop");
      esc.writeMicroseconds(prestop_throtle);
      while (!gtc.just_at_hi_pos()) {
        //Serial.print("g_just_at_hi_pos= "); Serial.println(int(g_just_at_hi_pos), DEC);
        digitalWrite(gtc.led_pin(), gtc.led_state());
      }
      gtc.reset_just_at_hi_pos();
      
      Serial.println("stop at hi pos");
      esc.writeMicroseconds(min_throtle);
      past_throtle = min_throtle;
      delay(2000);
      
      continue;
    }
   
    //
    esc.writeMicroseconds(throtle);
    delay(100);
  }
  
  return 0;
}
