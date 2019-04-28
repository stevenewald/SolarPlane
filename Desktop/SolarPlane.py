import time

import navio.ms5611
import navio.util

navio.util.check_apm()

baro = navio.ms5611.MS5611()
baro.initialize()

def MillibarToMg(millibar):
	return millibar*0.02953

print "initializing"


GravAcc = 9.807
MolarMassEarthAir = 0.02896
UniversalGasConstant = 8.3143

print "PUT BAROMETER IN SAFE ISOLATED PLACE"
time.sleep(15)

print "getting initial baro reading"

baro.refreshPressure()
time.sleep(0.01) # Waiting for pressure data ready 10ms
baro.readPressure()
baro.calculatePressureAndTemperature()

InitPressure = MillibarToMg(baro.PRES)

def HypsometricFormula(pres, temp):
	PressureInitOverCurrentPres = 1012.5/pres
	TempinKelvin = 273.15+temp
	return ((PressureInitOverCurrentPres**(1/5.257)-1)*TempinKelvin)/.0065
	

while(True):
	baro.refreshPressure()
	time.sleep(0.01) # Waiting for pressure data ready 10ms
	baro.readPressure()

	baro.refreshTemperature()
	time.sleep(0.01) # Waiting for temperature data ready 10ms
	baro.readTemperature()

	baro.calculatePressureAndTemperature()

	print "ALTITUDE: " + HyposometricFormula(baro.PRES, baro.TEMP)	

	time.sleep(1)
