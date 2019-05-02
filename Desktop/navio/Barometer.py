import time
import sys

import ms5611
import util

navio.util.check_apm()

baro = ms5611.MS5611()
baro.initialize()

def HypsometricFormula(pres, temp):
	PressureInitOverCurrentPres = 1012.5/pres
	TempinKelvin = 273.15+temp
	return ((PressureInitOverCurrentPres**(1/5.257)-1)*TempinKelvin)/.0065

def MetersToFeet(meters):
	return meters*3.28084

def deltaAltitude():
	return MetersToFeet(HypsometricFormula(baro.PRES, baro.TEMP)) - Offset

def BarometerReading():
	baro.refreshPressure()
	time.sleep(0.01) # Waiting for pressure data ready 10ms
	baro.readPressure()

	baro.refreshTemperature()
	time.sleep(0.01) # Waiting for temperature data ready 10ms
	baro.readTemperature()

	baro.calculatePressureAndTemperature()
	return MetersToFeet(HypsometricFormula(baro.PRES, baro.TEMP))
