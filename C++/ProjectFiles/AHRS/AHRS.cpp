//AHRS stands for atttitude and heading reference system



//TODO:
//Heading is consistant (changes by correct amount) but doesn't start at the correct heading
#include <Common/Ublox.h>
#include "Navio2/PWM.h"
#include <Navio2/Led_Navio2.h>
#include "Navio2/RCOutput_Navio2.h"
#include <Navio2/RCInput_Navio2.h>
#include <stdio.h>
#include <ratio>
#include <fstream>
#include <cstdio>
#include <memory>
#include <sys/socket.h>
#include <Common/MS5611.h>
#include <netinet/in.h> 
#include <arpa/inet.h>
#include <stdint.h>
#include <unistd.h>
#include <ctime>   
#include <chrono>
#include <thread>
#include <sys/time.h>
#include <Common/MPU9250.h>
#include <Navio2/LSM9DS1.h>
#include<iostream>
#include <Common/Util.h>
#include "AHRS.hpp"

#define G_SI 9.80665
#define PI   3.14159


/*void Loop() {
    while(check_apm()) {
        usleep(10000);
    }
}
Loop()*/

//for the rcinput
#define READ_FAILED -1


AHRS::AHRS(std::unique_ptr <InertialSensor> imu)
{
    sensor = move(imu);
    q0 = 1; q1 = 0; q2 = 0, q3 = 0; twoKi = 0; twoKp =2;
}
void AHRS::update(float dt)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;

    sensor->update();
        sensor->read_accelerometer(&ax, &ay, &az);
        sensor->read_gyroscope(&gx, &gy, &gz);
        sensor->read_magnetometer(&mx, &my, &mz);


    //use IMU algorithm if magnetometer measurement invalid (avoids NAN in magnetometer normalisation)
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        updateIMU(dt);
        return;
    }

    //compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        //normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        //normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        //auxiliary variables to avoid repeated calcs
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        //reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        //estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        //error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        //compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;	//integral error scaled by Ki
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx;	//apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f;	//prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        //apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    //integrate rate of change of quaternion
    gx *= (0.5f * dt);		//pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    //normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void AHRS::updateIMU(float dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    float ax, ay, az;
    float gx, gy, gz;

    //Accel + gyro.
    sensor->update();
    sensor->read_accelerometer(&ax, &ay, &az);
    sensor->read_gyroscope(&gx, &gy, &gz);

    ax /= G_SI;
    ay /= G_SI;
    az /= G_SI;
    gx *= (180 / PI) * 0.0175;
    gy *= (180 / PI) * 0.0175;
    gz *= (180 / PI) * 0.0175;

    gx -= gyroOffset[0];
    gy -= gyroOffset[1];
    gz -= gyroOffset[2];

    //compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        //normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        //estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        //error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        //compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;	//integral error scaled by Ki
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx;	//apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f;	// prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        //apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    //integrate rate of change of quaternion
    gx *= (0.5f * dt);		//pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    //normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void AHRS::setGyroOffset()
{
    //Calculate offset

    float offset[3] = {0.0, 0.0, 0.0};
    float gx, gy, gz;

    //MPU initialization

    sensor->initialize();

    //printf("Beginning gyro calibration...\n");
    for(int i = 0; i<100; i++)
    {
        sensor->update();
        sensor->read_gyroscope(&gx, &gy, &gz);

        gx *= 180 / PI;
        gy *= 180 / PI;
        gz *= 180 / PI;

        offset[0] += gx*0.0175;
        offset[1] += gy*0.0175;
        offset[2] += gz*0.0175;

        usleep(10000);
    }
    offset[0]/=100.0;
    offset[1]/=100.0;
    offset[2]/=100.0;

    //printf("Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);

    gyroOffset[0] = offset[0];
    gyroOffset[1] = offset[1];
    gyroOffset[2] = offset[2];
}

void AHRS::getEuler(float* roll, float* pitch, float* yaw)
{
   *roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2)) * 180.0/M_PI;
   *pitch = asin(2*(q0*q2-q3*q1)) * 180.0/M_PI;
   *yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3)) * 180.0/M_PI;
}

int AHRS::HypFormula(float pres, float temp)
{
    float PresInitOverCurrentPres;
    float TempInKelvin;


    PresInitOverCurrentPres = 1012.5/pres;
    TempInKelvin = 273.15+temp;

    return (((pow(PresInitOverCurrentPres, ((1/5.257)))-1)*TempInKelvin)/.0065)*3.28084;
    //return pres;
}


float AHRS::invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

float AHRS::getW()
{
    return  q0;
}

float AHRS::getX()
{
    return  q1;
}

float AHRS::getY()
{
    return  q2;
}

float AHRS::getZ()
{
    return  q3;
}


std::unique_ptr <InertialSensor> get_inertial_sensor( std::string sensor_name) //two different 9DOF imu's
{
    if (sensor_name == "mpu") {
        //printf("Selected: MPU9250\n");
        auto ptr = std::unique_ptr <InertialSensor>{ new MPU9250() };
        return ptr;
    }
    else if (sensor_name == "lsm") {
        printf("Selected: LSM9DS1\n");
        auto ptr = std::unique_ptr <InertialSensor>{ new LSM9DS1() };
        return ptr;
    }
    else {
        return NULL;
    }
}


std::string get_sensor_name(int argc, char *argv[])
{
    if (get_navio_version() == NAVIO2) {

        if (argc < 2) {
            printf("Enter parameter\n");
            return std::string();
        }

        //prevent the error message
        opterr = 0;
        int parameter;

        while ((parameter = getopt(argc, argv, "i:h")) != -1) {
            switch (parameter) {
            case 'i': return optarg;
            case '?': printf("Wrong parameter.\n");
                      return std::string();
            }
        }

    }
}

/*
//takes in current pitch and target pitch and returns corrected elevator servo
int AHRS::ElevatorCorrection(float pitch, float targetPitch)
{
    int newPitch;
    newPitch = pitch
}
*/

float longitude;
float latitude;
int gpsaccuracy;
auto led = std::unique_ptr <Led>{ new Led_Navio2() };

void pid_init(pid_ctrl_t *pid)
{
    pid_set_gains(pid, 1., 0., 0.);
    pid->integrator = 0.;
    pid->previous_error = 0.;
    pid->integrator_limit = INFINITY;
    pid->frequency = 1.;
}

void pid_set_gains(pid_ctrl_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_get_gains(const pid_ctrl_t *pid, float *kp, float *ki, float *kd)
{
    *kp = pid->kp;
    *ki = pid->ki;
    *kd = pid->kd;
}

float pid_get_integral_limit(const pid_ctrl_t *pid)
{
    return pid->integrator_limit;
}

float pid_get_integral(const pid_ctrl_t *pid)
{
    return pid->integrator;
}

float pid_process(pid_ctrl_t *pid, float error)
{
    float output;
    pid->integrator += error;

    if (pid->integrator > pid->integrator_limit) {
        pid->integrator = pid->integrator_limit;
    } else if (pid->integrator < -pid->integrator_limit) {
        pid->integrator = -pid->integrator_limit;
    }

    output  = - pid->kp * error;
    output += - pid->ki * pid->integrator / pid->frequency;
    output += - pid->kd * (error - pid->previous_error) * pid->frequency;

    pid->previous_error = error;
    return output;
}

void pid_set_integral_limit(pid_ctrl_t *pid, float max)
{
    pid->integrator_limit = max;
}

void pid_reset_integral(pid_ctrl_t *pid)
{
    pid->integrator = 0.;
}

void pid_set_frequency(pid_ctrl_t *pid, float frequency)
{
    pid->frequency = frequency;
}

float pid_get_frequency(const pid_ctrl_t *pid)
{
    return pid->frequency;
}



//============================== Main loop ====================================
using namespace std;
template <typename File> //idk the data type name lol
void imuLoop(AHRS* ahrs, int* phaseOfFlightVal, int* firstTimeRunningRcinput, int* printcounter, float* gyroCalibElev, File* t1, File* outputFile)
{
    *printcounter = *printcounter + 1;
    int inputElev;
    int inputRudd;
    int inputThrott;
    int inputSpoilers;
    int inputRealSpoilers;
    auto rcin = std::unique_ptr <RCInput>{ new RCInput_Navio2() };
    auto pwm = std::unique_ptr <RCOutput>{ new RCOutput_Navio2() };
    //orientation data
    MS5611 barometer;
    Ublox gps;
    if(*firstTimeRunningRcinput){
        barometer.initialize();
        rcin->initialize();
        led->initialize();
        //pwm->initialize(1);//throttle
        pwm->initialize(0);
        pwm->initialize(2);//elevator
        pwm->initialize(3);//rudder
        pwm->initialize(4);//spoiler
        pwm->initialize(5);//spoiler
        pwm->set_frequency(0, 50);
        pwm->set_frequency(2, 50);
        pwm->set_frequency(3, 50);
        pwm->set_frequency(4, 50);
        pwm->set_frequency(5, 50);
        
        
        *firstTimeRunningRcinput = false;
    }
    
    
    float roll, pitch, yaw;
    float altitudeInFeet;

    struct timeval tv;
    float dt;
    //timing data

    static float maxdt;
    static float mindt = 0.01;
    static float dtsumm = 0;
    static int isFirst = 1;
    static unsigned long previoustime, currenttime;
    


    //----------------------- Calculate delta time ----------------------------

    
	gettimeofday(&tv,NULL);
	previoustime = currenttime;
	currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
	dt = (currenttime - previoustime) / 1000000.0;
	if(dt < 1/1300.0) usleep((1/1300.0-dt)*1000000);
        gettimeofday(&tv,NULL);
        currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
	dt = (currenttime - previoustime) / 1000000.0;


    //--------PID Setup-------------------------------------------------------
    pid_ctrl_t pid;
    pid_init(&pid);

    //--------barometer measurements/altitude --------------------------------
    /*
    barometer.refreshPressure();
    usleep(10000); // Waiting for pressure data ready
    barometer.readPressure();

    barometer.refreshTemperature();
    usleep(10000); // Waiting for temperature data ready
    barometer.readTemperature();

    barometer.calculatePressureAndTemperature();

    altitudeInFeet = (ahrs->HypFormula(barometer.getPressure(), barometer.getTemperature()));
    //--------read raw measurements from the MPU and update AHRS--------------

    */
    ahrs->updateIMU(dt);
    


    //------------------------read euler angles------------------------------

    ahrs->getEuler(&roll, &pitch, &yaw);

    //-------------------discard the time of the first cycle-----------------

    if (!isFirst)
    {
    	if (dt > maxdt) maxdt = dt;
    	if (dt < mindt) mindt = dt;
    }
    isFirst = 0;

    //---------------- RCInput ----------------------------------------------

    inputThrott = rcin->read(0);
    inputRudd = rcin->read(1);
    inputElev = rcin->read(2);
    inputSpoilers = rcin->read(3);
    inputRealSpoilers = rcin->read(7);

    //Gyro calibration-------------------------------------------------------------
    if (*printcounter==10000){ //at 10000 ticks the gyro = 0
        *gyroCalibElev = roll; //bad fix for system but I don't have a much better option
    }
    
    /*
    std::vector<double> pos_data;
    if (gps.testConnection()){
    if (gps.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data) == 1)
    {
        // after desired message is successfully decoded, we can use the information stored in pos_data vector
        // right here, or we can do something with it from inside decodeSingleMessage() function(see ublox.h).
        // the way, data is stored in pos_data vector is specified in decodeMessage() function of class UBXParser(see ublox.h)
        //printf("GPS Millisecond Time of Week: %.0lf s\n", pos_data[0]/1000);
        longitude = pos_data[1]/10000000;
        latitude = pos_data[2]/10000000;
        //printf("Height above Ellipsoid: %.3lf m\n", pos_data[3]/1000);
        //printf("Height above mean sea level: %.3lf m\n", pos_data[4]/1000);
        gpsaccuracy = pos_data[5]/1000;
        //printf("Vertical Accuracy Estateimate: %.3lf m\n", pos_data[6]/1000);
        // printf("Message not captured\n");
        // use this to see, how often you get the right messages
        // to increase the frequency you can turn off the undesired messages or tweak ublox settings
        // to increase internal receiver frequency
    }
    }
    */


    //printf("inputelev:");
    //cout << inputRudd;
    //manualoverride = rcin->read(3)

    //startup sequence
    if(*phaseOfFlightVal==1)
    {
    pwm->set_duty_cycle(2, 1250);
    pwm->set_duty_cycle(3, 1250);
    usleep(250000);
    pwm->set_duty_cycle(2, 1750);
    pwm->set_duty_cycle(3, 1750);
    usleep(250000);
    pwm->set_duty_cycle(2, 1250);
    pwm->set_duty_cycle(3, 1250);
    usleep(250000);
    pwm->set_duty_cycle(2, 1750);
    pwm->set_duty_cycle(3, 1750);
    usleep(250000);
    pwm->set_duty_cycle(2, 1250);
    pwm->set_duty_cycle(3, 1250);
    usleep(250000);
    pwm->set_duty_cycle(2, 1750);
    pwm->set_duty_cycle(3, 1750);
    usleep(250000);
    pwm->set_duty_cycle(2, 1250);
    pwm->set_duty_cycle(3, 1250);
    usleep(250000);
    pwm->set_duty_cycle(2, 1500);
    pwm->set_duty_cycle(3, 1500);
    *phaseOfFlightVal = 3; //CHANGE
    }


    //if gps is accurate, flashes green
    if(*phaseOfFlightVal==2)
    {
        if(gpsaccuracy<7){
            led->setColor(Colors::Green);
            usleep(1000000);
            led->setColor(Colors::Black);
            usleep(1000000);
            led->setColor(Colors::Green);
            usleep(1000000);
            led->setColor(Colors::Black);
            usleep(1000000);
            led->setColor(Colors::Green);
            usleep(1000000);
            led->setColor(Colors::Black);
            usleep(1000000);
            *phaseOfFlightVal = 3;
        } else {
            led->setColor(Colors::Red);
        }
    }
 
    float elevatorComp;
    //elevatorComp = (pow(abs(roll), 1.2));
    pid_set_gains(&pid, 2., 0.01, 0.001);
    elevatorComp = pid_process(&pid, (-1*roll+*gyroCalibElev));
    if(roll > 0)  //this is for the non-pid controller, is redundant with it
    {
        elevatorComp = ((1.5+(elevatorComp)/100)*1000); //formatting for servo duty cycle
    }
    else
    {
        //elevatorComp = ((1.5-(elevatorComp)/100)*1000); //221 original
        elevatorComp = ((1.5+(elevatorComp)/100)*1000); //for PID
    }
    
    if(*phaseOfFlightVal==3) 
    {
        if(inputSpoilers>1500)
        {
            pwm->set_duty_cycle(2, elevatorComp);
            pwm->set_duty_cycle(3, inputRudd);
            pwm->set_duty_cycle(0, inputThrott);
            pwm->set_duty_cycle(5, inputRealSpoilers);
        } else {
            pwm->set_duty_cycle(2, inputElev);
            pwm->set_duty_cycle(3, inputRudd);
            pwm->set_duty_cycle(0, inputThrott);
            pwm->set_duty_cycle(5, inputRealSpoilers);
        }
        //pwm->set_duty_cycle(3, inputRudd);

    } 
    //pwm->set_duty_cycle(4, inputSpoilers);

    //--------------Compensation/servoupdates-----------------------










    //-------------console and network output with a lowered rate------------
    
    //Calculate altitude in feet
    auto coutTime = std::chrono::system_clock::now(); //time in seconds
    std::time_t end_time = std::chrono::system_clock::to_time_t(coutTime);


    dtsumm += dt;
    //if(dtsumm > 0.05)
    if(remainder(*printcounter, 200) == 0)
    {
        // Console output
        cout << (roll) << endl;
        cout << pitch << endl;
        cout << (yaw * -1) << endl;

        using namespace std::chrono;

        high_resolution_clock::time_point t2 = high_resolution_clock::now();

        duration<double, std::milli> time_span = t2 - *t1;

        using namespace std;
        *outputFile << time_span.count() << endl;
        *outputFile << roll << endl;
        *outputFile << elevatorComp << endl;
 
        dtsumm = 0;
    }
    //std::this_thread::sleep_for(std::chrono::milliseconds(20)); //prevent overflow of network (idk, happened in the beginning a bit but maybe due to worse
    //programming at the time
    //which of course, was definitely and absolutely fixed by now
    //but we wont test it anyway)
}
//-------------------------- Logging data -------------------------------------


//=============================================================================
using namespace std;
int main(int argc, char *argv[])
{   

    int phaseOfFlightVal;
    int printcounter;
    
    printcounter = 0;

    int firstTimeRunningRcinput;
    if (check_apm()) {
        return 1;
    }
    

    auto sensor_name = get_sensor_name(argc, argv);

    if (sensor_name.empty())
        return EXIT_FAILURE;

    auto imu = get_inertial_sensor(sensor_name);

    if (!imu) {
        printf("Wrong sensor name. Select: mpu or lsm\n"); //can use both IMUs
        return EXIT_FAILURE;
    }

    if (!imu->probe()) {
        printf("Sensor not enable\n"); //sometimes it doesn't work, idk why but its an imu issue not programming (at least, not MY programming. maybe the imu package made by the imu devs)
        return EXIT_FAILURE;
    }
    phaseOfFlightVal = 1;

    //---------------------------network setup-------------------------------

    auto ahrs = std::unique_ptr <AHRS>{new AHRS(move(imu)) };

    using namespace std::chrono;
    high_resolution_clock::time_point t1 = high_resolution_clock::now(); //starting time
    using namespace std;

    //File output
    ifstream inputFile("input.txt");
    inputFile.open("input.txt");
    ofstream outputFile("output.txt");
    outputFile.open("output.txt");
    inputFile.tie(&outputFile);

    //--------------------setup gyroscope offset-----------------------------
    float gyroCalibElev;
    firstTimeRunningRcinput = true;
    ahrs->setGyroOffset();
    while(1)
        imuLoop(ahrs.get(), &phaseOfFlightVal, &firstTimeRunningRcinput, &printcounter, &gyroCalibElev, &t1, &outputFile);
}
