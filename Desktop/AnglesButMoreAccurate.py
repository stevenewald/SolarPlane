import time
import sys
import navio.ms5611
import navio.util
import argparse 
import navio.mpu9250
import navio.rcinput
import math
import navio.pwm
import datetime
import os
from collections import namedtuple

navio.util.check_apm()

imu = navio.mpu9250.MPU9250()

currenttime = 0
previoustime = 0

if imu.testConnection():
    print("Connection established: True")
else:
    sys.exit("Connection established: False")

imu.initialize()

gyroOffset = [0, 0, 0]

time.sleep(1)

G_SI = 9.80665
PI = 3.14159
M_PI = 3.14159

q0 = 1
q1 = 0
q2 = 0
q3 = 0
twoKi = 0
twoKp = 2
recipNorm = 0
q0q0=q0q1=q0q2=q0q3=q1q1=q1q2=q1q3=q2q2=q2q3=q3q3 = 0
hx=hy=bx=bz = 0
halfvx=halfvy=halfvz=halfwx=halfwy=halfwz = 0
halfex=halfey=halfez = 0
qa=qb=qc = 0
ax=ay=az = 0
gx=gy=gz = 0
mx=my=mz = 0
pitch=roll=yaw = 0

#dt = deltatime

def invSqrt(x):
    return x**-0.5

def Update(dt):
    global recipNorm, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3
    global hx, hy, bz, bz
    global halfvx, halfvy, halfvz, halfwx, halfwy, halfwz
    global halfex, halfey, halfez
    global qa, qb, qc
    global ax, ay, az
    global qx, qy, qz
    global mx, my, mz
    global q0,q1,q2,q3

    ###### Establishing variables but grouping similarly used ones
    m9a, m9g, m9m = imu.getMotion9()
    #Read the accelerometer,gyroscope and magnetometer values
    #And apply them to the correct values
    ax = m9a[0]
    ay = m9a[1]
    az = m9a[2]
    gx = m9g[0]
    gy = m9g[1]
    gz = m9g[2]
    mx = m9m[0]
    my = m9m[1]
    mz = m9m[2]

    #Use IMU algorithm if magnetometer measurement invalid
    if (((mx == 0.0) and (my == 0.0)) and (mz == 0.0)):
        updateIMU(dt)
    #Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)

    if not(((ax == 0.0) and (ay == 0.0)) and (az == 0.0)):
        #normalise accelerometer measurement
        recipNorm = (ax * ax + ay * ay + az * az)**-.05 #inverse squareroot
        ax = ax*recipNorm
        ay = ay*recipNorm
        az = az*recipNorm

        #normalise magnetometer measurement
        recipNorm = (mx * mx + my * my + mz * mz)
        mx = mx*recipNorm
        my = my*recipNorm
        mz = mz*recipNorm

        #auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0
        q0q1 = q0 * q1
        q0q2 = q0 * q2
        q0q3 = q0 * q3
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q3q3 = q3 * q3

        #reference direction of earths magnetic field
        hx = 2 * (mx * (0.5 - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2))
        hy = 2 * (mx * (q1q2 + q0q3) + my * (0.5 - q1q1 - q3q3) + mz * (q2q3 - q0q1))
        bx = math.sqrt(hx * hx + hy * hy)
        bz = 2 * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5 - q1q1 - q2q2))

        #estimated direction of gravity and magnetic field
        #what does this even mean anymore, im just trying to interpret this
        #and i have no clue what im doing ffs
        halfvx = q1q3 - q0q2
        halfvy = q0q1 + q2q3
        halfvz = q0q0 - 0.5 + q3q3
        halfwx = bx * (0.5 - q2q2 - q3q3) + bz * (q1q3 - q0q2)
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3)
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5 - q1q1 - q2q2)

        #error is sum of cross product between estimated direction and measured direction fo field vector
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy)
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz)
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx)

        if (twoKi > 0):
            integralFBx = integralFbx + twoKi * halfex * dt #integral error scaled by one
            integralFBy = integralFBy + twoKi * halfey * dt
            integralFBz = integralFBz + twoKi * halfez * dt
            gx = gx + integralFBx #apply integral feedback
            gy = gy + integralFBy
            gz = gz + integralFBz
        else:
            integralFBx = 0 #prevent integral windup
            integralFBy = 0
            integralFBz = 0
        
        #apply proportional feedback
        gx = gx + twoKp * halfex
        gy = gy + twoKp * halfey
        gz = gz + twoKp * halfez

    #integrate rate of change of quaternion

    gx = gx*(0.5 * dt) #pre multiply common factors
    gy = gy*(0.5 * dt)
    gz = gz*(0.5 * dt)
    ga = q0
    qb = q1
    qc = q2
    q0 = q0 + (-qb * gx - qc * gy - q3 * gz)
    q1 = q1 + (qa * gx + qc * gz - q3 * gy)
    q2 = q2 + (qa * gy - qb * gz + q3 * gx)
    q3 = q3 + (qa * gz + qb * gy - qc * gx)

    #normalize quaternion
    recipNorm = (q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)**-0.5 #invsqrt
    q0 = q0*recipNorm
    q1 = q1*recipNorm
    q2 = q2*recipNorm
    q3 = q3*recipNorm

def updateIMU(dt):
    global recipNorm, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3
    global halfvx, halfvy, halfvz
    global halfex, halfey, halfez
    global qa, qb, qc
    global ax, ay, az
    global gx, gy, gz
    global q0,q1,q2,q3
    global gyroOffset
    global twoKi


    m9a, m9g, m9m = imu.getMotion9()
    #Read the accelerometer,gyroscope and magnetometer values
    #And apply them to the correct values
    ax = m9a[0]
    ay = m9a[1]
    az = m9a[2]
    gx = m9g[0]
    gy = m9g[1]
    gz = m9g[2]

    ax=ax/G_SI
    ay=ay/G_SI
    az=az/G_SI

    gx=gx*(180/PI)*0.0175
    gy=gy*(180/PI)*0.0175
    gz=gz*(180/PI)*0.0175

    gx = gx-gyroOffset[0]
    gy = gy-gyroOffset[1]
    gz = gz-gyroOffset[2]

    #Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if not(((ax == 0) and (ay == 0)) and (az == 0)):
        #normalize accelerometer measurement
        recipNorm = (ax*ax + ay*ay + az*az)
        ax=ax*recipNorm
        ay=ay*recipNorm
        az=az*recipNorm

        #estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1*q3 - q0*q2
        halfvy = q0*q1 + q2*q3
        halfvz = q0*q0 - 0.5+q3*q3

        #error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy)
        halfey = (az * halfvx - ax * halfvz)
        halfez = (ax * halfvy - ay * halfvx)

        #compute and apply integral feedback if enabled
        if (twoKi > 0):
            integralFBx =+ twoKi * halfex * dt	#integral error scaled by Ki
            integralFBy =+ twoKi * halfey * dt
            integralFBz =+ twoKi * halfez * dt
            gx =+ integralFBx	#apply integral feedback
            gy =+ integralFBy
            gz =+ integralFBz
        else:
            integralFBx = 0  #prevent integral windup
            integralFBy = 0
            integralFBz = 0

        #apply proportional feedback
        gx =+ twoKp * halfex
        gy =+ twoKp * halfey
        gz =+ twoKp * halfez

        #integrate rate of change of quaternion
        gx=gx*(0.5*dt)
        gy=gy*(0.5*dt)
        gz=gz*(0.5*dt)
        qa=q0
        qb=q1
        qc=q2
        q0 =+ (-qb * gx - qc * gy - q3 * gz)
        q1 =+ (qa * gx + qc * gz - q3 * gy)
        q2 =+ (qa * gy - qb * gz + q3 * gx)
        q3 =+ (qa * gz + qb * gy - qc * gx)

        #normalize quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        q0 = q0 * recipNorm
        q1 = q1 * recipNorm
        q2 = q2 * recipNorm
        q3 = q3 * recipNorm


def getEuler():
    global q0,q1,q2,q3
    global pitch,roll,yaw

    roll = math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2)) * 180.0/M_PI
    pitch = math.asin(2*(q0*q2-q3*q1)) * 180.0/M_PI
    yaw = math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3)) * 180.0/M_PI









def setGyroOffset():
    offset = [0, 0, 0]
    global gx, gy, gz
    gx=gy=gz = 0

    print("beginning gyro calibration")
    i = 0
    while (i < 100):
        m9a, m9g, m9m = imu.getMotion9()
        #Read the accelerometer,gyroscope and magnetometer values
        #And apply them to the correct values
        gx = m9g[0]
        gy = m9g[1]
        gz = m9g[2]

        gx = gx * (180/PI)
        gy = gy * (180/PI)
        gz = gz * (180/PI)

        offset[0] = offset[0] + gx*0.0175
        offset[1] = offset[1] + gy*0.0175
        offset[2] = offset[2] + gz*0.0175
        i = i + 1
        time.sleep(.001)

    offset[0] = offset[0]/100
    offset[1] = offset[1]/100
    offset[2] = offset[2]/100

    print("offsets are: " + str(offset[0]) + " " + str(offset[1]) + " " + str(offset[2]))

    global gyroOffset
    gyroOffset[0] = offset[0]
    gyroOffset[1] = offset[1]
    gyroOffset[2] = offset[2]

def mainLoop():
    global roll, pitch, yaw
    global previoustime
    global currenttime


    dt = 0
    #struct timeval tv
    #timing data

    maxdt = 0
    mindt = 0.01
    dtsumm = 0
    isFirst = 1
    currenttime = 0

    now = time.time()
    previoustime = currenttime
    currenttime = now
    dt = (currenttime - previoustime)
    if (dt < 1/1300):
        time.sleep(1/1300-dt)
        now = time.time()
        currenttime = now
        dt = (currenttime - previoustime)
    getEuler()
    Update(dt)
    print("PITCH, ROLL, YAW: " + str(pitch) + " " +  str(roll) + " " + str(yaw))
setGyroOffset()
n = 0
while n < 100:
    mainLoop()
    n = n + 1






