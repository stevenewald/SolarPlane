import time

import navio.ms5611
import navio.util

navio.util.check_apm()

baro = navio.ms5611.MS5611()
baro.initialize()

baro.refreshPressure()
	time.sleep(0.01) # Waiting for pressure data ready 10ms
	baro.readPressure()

print "initializing"
baro.refreshPressure()
time.sleep(0.01) # Waiting for pressure data ready 10ms
baro.readPressure()


GravAcc = 9.807
MolarMassEarthAir = 0.02896
UniversalGasConstant = 8.3143

time.Sleep(5)

InitPressure = baro.PRES

while(True):
	baro.refreshPressure()
	time.sleep(0.01) # Waiting for pressure data ready 10ms
	baro.readPressure()

	baro.refreshTemperature()
	time.sleep(0.01) # Waiting for temperature data ready 10ms
	baro.readTemperature()

	baro.calculatePressureAndTemperature()

	print "Temperature(C): %.6f" % (baro.TEMP), "Pressure(millibar): %.6f" % (baro.PRES)

	ChangeInPres = (InitPressure - baro.PRES)
	KelvinTemp = 273.15 + baro.TEMP
	ChangeInHeight = (ChangeInPres*KelvinTemp)/baro.PRES
	print "BAROPRES:"
	print ChangeInHeight

	time.sleep(1)