#include <arduino-core/Arduino.h>
#include <sensor/chr_um6.hpp>

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  Serial.begin(115200);
  
  crim::CHR_UM6 um6;
  const double kOneRadianInDegree = 57.2957795;
  
  //while (true) {
    //Serial.println("============================================================");
    //Serial.println(um6.firmware_version());
    
    //delay(1000);
  //}
  
  Serial.println("calib: BEGIN");
  um6.calib();
  Serial.println("calib: END");
  
  while (true) {
    crim::EulerAngle euler;
    euler = um6.euler();
    
    Serial.println("---------------------------------------------------------");
    Serial.print("euler.roll= "); Serial.println(euler.roll*kOneRadianInDegree);
    Serial.print("euler.pitch= "); Serial.println(euler.pitch*kOneRadianInDegree);
    Serial.print("euler.yaw= "); Serial.println(euler.yaw*kOneRadianInDegree);
    
    delay(100);
  }
  
  Serial.end();
  return 0;
}
