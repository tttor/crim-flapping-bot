#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "matrix/vector.h"
#include "sensor/sensor.h"
#include "sensor/accelerometer.h"
#include "sensor/compass.h"
#include "sensor/gyroscope.h"
#include "attitude/attitude.h"

namespace crim {
  
class Attitude {
 private:
	static Vector<double> X;
	static Vector<double> Y;
	static Vector<double> Z;
	static unsigned int timestamp;
	
 public:
  Compass compass;
  Accelerometer accelerometer;
  Gyroscope gyroscope;
  
  Attitude();
	~Attitude();
	void getXYZ(Vector<double> & X,Vector<double> & Y,Vector<double> & Z);
	Matrix<double> getRPY(double & roll,double & pitch,double & yaw);
	Vector<double> getQuaternion();
  void getZ(Vector<double> & newZ,Vector<double> & deltaTheta);
	void getX(const Vector<double> & newZ,Vector<double> & newX,Vector<double> & deltaTheta);
	void getY(const Vector<double> & newX,const Vector<double> & newZ,Vector<double> & newY);
	void update(const Vector<double> & newX,const Vector<double> & newY,const Vector<double> & newZ);
  
};

} // namespace crim
#endif
