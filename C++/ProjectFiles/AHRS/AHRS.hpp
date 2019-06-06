#ifndef AHRS_HPP
#define AHRS_HPP

#include <cmath>
#include <stdio.h>
#include <memory>
#include <Common/InertialSensor.h>

class AHRS{
private:
	float q0, q1, q2, q3;
	float gyroOffset[3];
    int firstTimeRunningAlt;
    int firstTimeRunningRcinput;
	float twoKi;
	float twoKp;
    int alttiude;
    float inputElev;
    float inputRudd;
    float inputThrott;
    float inputSpoilers;
    //auto rcin;
   // auto pwm;
    //float mx, my; //remove
	float integralFBx, integralFBy, integralFBz;
    std::unique_ptr <InertialSensor> sensor;
public:
    AHRS( std::unique_ptr <InertialSensor> imu);

    void update(float dt);
    void updateIMU(float dt);
    void setGyroOffset();
    void getEuler(float* roll, float* pitch, float* yaw);
    //void updateServos(float Elev, float Rudd, float Thrott, float Spoilers);

    int HypFormula(float pres, float temp);
    float invSqrt(float x);
    float getW();
    float getX();
    float getY();
    float getZ();
};

#endif // AHRS_hpp
