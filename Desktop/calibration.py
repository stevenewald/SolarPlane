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

a = datetime.datetime.now()

navio.util.check_apm()

imu = navio.mpu9250.MPU9250()

imu.initialize()

#Preload the variables used to keep track of the minimum and maximum values
magXmin = 32767
magYmin = 32767
magZmin = 32767
magXmax = -32767
magYmax = -32767
magZmax = -32767


while True:

    #Read magnetometer values
    m9a, m9g, m9m = imu.getMotion9()

    MAGx = m9m[0]
    MAGy = m9m[1]
    MAGz = m9m[2]
    
    
    
    if MAGx > magXmax:
        magXmax = MAGx
    if MAGy > magYmax:
        magYmax = MAGy
    if MAGz > magZmax:
        magZmax = MAGz

    if MAGx < magXmin:
        magXmin = MAGx
    if MAGy < magYmin:
        magYmin = MAGy
    if MAGz < magZmin:
        magZmin = MAGz

    print(" magXmin  %i  magYmin  %i  magZmin  %i  ## magXmax  %i  magYmax  %i  magZmax %i  " %(magXmin,magYmin,magZmin,magXmax,magYmax,magZmax))



    #slow program down a bit, makes the output more readable
    time.sleep(0.03)
