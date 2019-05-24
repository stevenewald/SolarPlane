import time
import sys
import navio.ms5611
import navio.util
import argparse 
import navio.mpu9250
import navio.rcinput
import math
import navio.pwm

navio.util.check_apm()

imu = navio.mpu9250.MPU9250()

if imu.testConnection():
    print("Connection established: True")
else:
    sys.exit("Connection established: False")

imu.initialize()

while True:
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
    compass = math.atan2(my, mz)
    compassdegrees = compass*57.29578
    print(compassdegrees)
    #print("mx " + str(mx))
    #print("my " + str(my))
    #print("mz " + str(mz))