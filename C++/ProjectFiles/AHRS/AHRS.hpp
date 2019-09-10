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
    int printcounter;
    float gyroCalibElev;
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
    int phaseOfFlightVal;
    float longitude;
    float latitude;
    //void updateServos(float Elev, float Rudd, float Thrott, float Spoilers);
    int HypFormula(float pres, float temp);
    float invSqrt(float x);
    float getW();
    float getX();
    float getY();
    float getZ();
};


//PID Controller stuff
typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator;
    float previous_error;
    float integrator_limit;
    float frequency;
} pid_ctrl_t;

/** Initializes a PID controller. */
void pid_init(pid_ctrl_t *pid);

/** Sets the gains of the given PID. */
void pid_set_gains(pid_ctrl_t *pid, float kp, float ki, float kd);

/** Returns the proportional gains of the controller. */
void pid_get_gains(const pid_ctrl_t *pid, float *kp, float *ki, float *kd);

/** Returns the limit of the PID integrator. */
float pid_get_integral_limit(const pid_ctrl_t *pid);

/** Returns the value of the PID integrator. */
float pid_get_integral(const pid_ctrl_t *pid);

/** Process one step if the PID algorithm. */
float pid_process(pid_ctrl_t *pid, float error);

/** Sets a maximum value for the PID integrator. */
void pid_set_integral_limit(pid_ctrl_t *pid, float max);

/** Resets the PID integrator to zero. */
void pid_reset_integral(pid_ctrl_t *pid);

/** Sets the PID frequency for gain compensation. */
void pid_set_frequency(pid_ctrl_t *pid, float frequency);

/** Gets the PID frequency for gain compensation. */
float pid_get_frequency(const pid_ctrl_t *pid);

#endif // AHRS_hpp
