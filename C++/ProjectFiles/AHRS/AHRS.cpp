//I credit berryIMU for the majority of the concepts here (https://github.com/ozzmaker/BerryIMU)
//Not my IMU but the concepts helped a lot (even if I had to spend double the time on the tilt compensation afterwards, including
//rewriting in C++ from scratch after 15+ hours of work bc of threading reasons)
//AHRS stands for atttitude and heading reference system



//TODO:
//Heading is consistant (changes by correct amount) but doesn't start at the correct heading

#include <Common/Ublox.h>
#include "Navio2/PWM.h"
#include <Navio2/Led_Navio2.h>
#include "Navio2/RCOutput_Navio2.h"
#include <Navio2/RCInput_Navio2.h>
#include <stdio.h>
#include <memory>
#include <sys/socket.h>
#include <Common/MS5611.h>
#include <netinet/in.h> 
#include <arpa/inet.h>
#include <stdint.h>
#include <unistd.h>
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



//============================== Main loop ====================================
using namespace std;
void imuLoop(AHRS* ahrs, int* phaseOfFlightVal, int* firstTimeRunningRcinput)
{
    int inputElev;
    int inputRudd;
    int inputThrott;
    int inputSpoilers;
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
        pwm->initialize(2);//elevator
        pwm->initialize(3);//rudder
        pwm->initialize(4);//spoiler
        //pwm->set_frequency(1, 50);
        pwm->set_frequency(2, 50);
        pwm->set_frequency(3, 50);
        pwm->set_frequency(4, 50);
        
        
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

    //--------barometer measurements/altitude --------------------------------
    barometer.refreshPressure();
    usleep(10000); // Waiting for pressure data ready
    barometer.readPressure();

    barometer.refreshTemperature();
    usleep(10000); // Waiting for temperature data ready
    barometer.readTemperature();

    barometer.calculatePressureAndTemperature();

    altitudeInFeet = (ahrs->HypFormula(barometer.getPressure(), barometer.getTemperature()));
    //--------read raw measurements from the MPU and update AHRS--------------

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

    inputRudd = rcin->read(1);
    inputElev = rcin->read(2);
    inputSpoilers = rcin->read(3);
    inputThrott = rcin->read(0);
    
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
    *phaseOfFlightVal = 2;
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

    int elevatorCompensation;
    if(*phaseOfFlightVal==3)
    {
        elevatorCompensation = (1.5+(pow(pitch, 1.2))/221);
        pwm->set_duty_cycle(2, elevatorCompensation);
        //pwm->set_duty_cycle(3, inputRudd);

    }
    //pwm->set_duty_cycle(4, inputSpoilers);

    //--------------Compensation/servoupdates-----------------------










    //-------------console and network output with a lowered rate------------
    
    //Calculate altitude in feet
    

    dtsumm += dt;
    if(dtsumm > 0.05)
    {
        // Console output

        //cout << roll << endl;
        cout << elevatorCompensation << endl;
        cout << pitch << endl;
        cout << (yaw * -1) << endl;

        dtsumm = 0;
    }
    //std::this_thread::sleep_for(std::chrono::milliseconds(20)); //prevent overflow of network (idk, happened in the beginning a bit but maybe due to worse
    //programming at the time
    //which of course, was definitely and absolutely fixed by now
    //but we wont test it anyway)
}

//=============================================================================
using namespace std;
int main(int argc, char *argv[])
{   

    int phaseOfFlightVal;

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

    //--------------------setup gyroscope offset-----------------------------

    firstTimeRunningRcinput = true;
    ahrs->setGyroOffset();
    while(1)
        imuLoop(ahrs.get(), &phaseOfFlightVal, &firstTimeRunningRcinput);
}
