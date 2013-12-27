#include <cmath>
#include <limits>
#include "wirish_time.h"
#include "sensor/accelerometer.h"
#include "sensor/compass.h"
#include "sensor/gyroscope.h"
#include "attitude/attitude.h"

using namespace crim;
using std::sqrt;
using std::numeric_limits;

Vector<double> Attitude::X(3);
Vector<double> Attitude::Y(3);
Vector<double> Attitude::Z(3);
unsigned int Attitude::timestamp;

Attitude::Attitude() {
	Vector<double> dTheta;
	getZ(Z,dTheta);
	getX(Z,X,dTheta);
	getY(X,Z,Y);
	timestamp = micros();
}

Attitude::~Attitude() {
}

void Attitude::getZ(Vector<double> & newZ,Vector<double> & deltaTheta) {

	Vector<double> accOfG = accelerometer.getReading();
	double modulus = sqrt(accOfG(0) * accOfG(0) + accOfG(1) * accOfG(1) + accOfG(2) * accOfG(2));
	newZ = Vector<double>(3);
  if (modulus > 0) {
    newZ(0) = -accOfG(0) / modulus;
    newZ(1) = -accOfG(1) / modulus;
    newZ(2) = -accOfG(2) / modulus;
  }
	Vector<double> dZ = newZ - Z;
	deltaTheta = cross_prod(Z,dZ);
}

void Attitude::getX(const Vector<double> & newZ,Vector<double> & newX,Vector<double> & deltaTheta) {
  
	Vector<double> northMagneticPole = compass.getReading();
	Vector<double> north = northMagneticPole;
  
	double offset = inner_prod(newZ,north) / inner_prod(newZ,newZ);
	north = north - offset * newZ;
	double modulus = sqrt(north(0) * north(0) + north(1) * north(1) + north(2) * north(2));

  newX = Vector<double>(3);
  if( modulus > 0) {
    newX(0) = north(0) / modulus;
    newX(1) = north(1) / modulus;
    newX(2) = north(2) / modulus;
  }
  
  Vector<double> dX = newX - X;

  deltaTheta = cross_prod(X,dX);
}

void Attitude::getY(const Vector<double> & newX,const Vector<double> & newZ,Vector<double> & newY) {
	newY = cross_prod(newZ,newX);
	double modulus = sqrt(newY(0) * newY(0) + newY(1) * newY(1) + newY(2) * newY(2));
  if(modulus > 0) {
    newY(0) = newY(0) / modulus;
    newY(1) = newY(1) / modulus;
    newY(2) = newY(2) / modulus;
  }
}

inline void Attitude::update(const Vector<double> & newX,const Vector<double> & newY,const Vector<double> & newZ) {
	X = newX; Y = newY; Z = newZ;
}

void Attitude::getXYZ(Vector<double> & retValX, Vector<double> & retValY, Vector<double> & retValZ) {
	unsigned int newtimestamp = micros();
	double dt = ((newtimestamp > timestamp)?(newtimestamp - timestamp):(numeric_limits<unsigned int>::max() - timestamp + newtimestamp)) * 1.0 / 1e6;
  
	Vector<double> newZ,newX,newY;
	Vector<double> dThetaComp,dThetaAcc,dThetaGyro;
  
	getZ(newZ,dThetaAcc);
	getX(newZ,newX,dThetaComp);
	getY(newX,newZ,newY);
  
	Vector<double> angularSpd = gyroscope.getReading();
	dThetaGyro = angularSpd * dt;

	Vector<double> sum;
	sum = dThetaAcc + dThetaComp;
	sum = sum + dThetaGyro;
	Vector<double> dTheta = sum / 3;

	Vector<double> dX,dY,dZ;
	dX = cross_prod(dTheta,X);
	dY = cross_prod(dTheta,Y);
	dZ = cross_prod(dTheta,Z);
  
	retValX = X + dX; retValY = Y + dY; retValZ = Z + dZ;

	update(newX,newY,newZ);
	timestamp = micros();
}

Matrix<double> Attitude::getRPY(double & roll,double & pitch,double & yaw) {
	Matrix<double> DCM(3,3);
	Vector<double> newX,newY,newZ;

  getXYZ(newX,newY,newZ);
  
	DCM(0,0) = newX(0);	DCM(0,1) = newX(1);	DCM(0,2) = newX(2);
	DCM(1,0) = newY(0);	DCM(1,1) = newY(1);	DCM(1,2) = newY(2);
	DCM(2,0) = newZ(0);	DCM(2,1) = newZ(1);	DCM(2,2) = newZ(2);

	roll = -atan2(DCM(2,1),DCM(2,2));
	pitch = asin(DCM(2,0));
	yaw = atan2(DCM(1,0),DCM(0,0));
	return DCM;
}

Vector<double> Attitude::getQuaternion() {
	Vector<double> quaternion(4);
	Vector<double> newX,newY,newZ;
	getXYZ(newX,newY,newZ);
	Matrix<double> DCM(3,3);
	DCM(0,0) = newX(0);	DCM(0,1) = newX(1);	DCM(0,2) = newX(2);
	DCM(1,0) = newY(0);	DCM(1,1) = newY(1);	DCM(1,2) = newY(2);
	DCM(2,0) = newZ(0);	DCM(2,1) = newZ(1);	DCM(2,2) = newZ(2);
	double trace = DCM(0,0) + DCM(1,1) + DCM(2,2);
	if(trace > 0) {
		double s = 0.5 / sqrt(trace + 1.0);
		quaternion(0) = 0.25 / s;
		quaternion(1) = (DCM(2,1) - DCM(1,2)) * s;
		quaternion(2) = (DCM(0,2) - DCM(2,0)) * s;
		quaternion(3) = (DCM(1,0) - DCM(0,1)) * s;
	} else {
		if(DCM(0,0) > DCM(1,1) && DCM(0,0) > DCM(2,2)) {
			double s = 2.0 * sqrt(1.0 + DCM(0,0) - DCM(1,1) - DCM(2,2));
      if(s > 0) {
        quaternion(0) = (DCM(2,1) - DCM(1,2)) / s;
        quaternion(1) = 0.25 * s;
        quaternion(2) = (DCM(0,1) + DCM(1,0)) / s;
        quaternion(3) = (DCM(0,2) + DCM(2,0)) / s;
      }
		} else if(DCM(1,1) > DCM(2,2)) {
			double s = 2.0 * sqrt(1.0 + DCM(1,1) - DCM(0,0) - DCM(2,2));
      if(s > 0) {
        quaternion(0) = (DCM(0,2) - DCM(2,0)) / s;
        quaternion(1) = (DCM(0,1) - DCM(1,0)) / s;
        quaternion(2) = 0.25 * s;
        quaternion(3) = (DCM(1,2) + DCM(2,1)) / s;
      }
		} else {
			double s = 2.0 * sqrt(1.0 + DCM(2,2) - DCM(0,0) - DCM(1,1));
      if(s > 0) {
        quaternion(0) = (DCM(1,0) - DCM(0,1)) / s;
        quaternion(1) = (DCM(0,2) + DCM(2,0)) / s;
        quaternion(2) = (DCM(1,2) + DCM(2,1)) / s;
        quaternion(3) = 0.25 * s;
      }
		}
	}
	
	return quaternion;
}
