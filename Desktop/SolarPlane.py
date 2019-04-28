import time
import sys

import navio.ms5611
import navio.util
import navio.leds

navio.util.check_apm()

led = navio.leds.Led()
baro = navio.ms5611.MS5611()
baro.initialize()

print "initializing"

print "PUT BAROMETER IN SAFE ISOLATED PLACE"
time.sleep(10)

print "getting initial baro reading"

baro.refreshPressure()
time.sleep(0.01) # Waiting for pressure data ready 10ms
baro.readPressure()
baro.calculatePressureAndTemperature()

def HypsometricFormula(pres, temp):
	PressureInitOverCurrentPres = 1012.5/pres
	TempinKelvin = 273.15+temp
	return ((PressureInitOverCurrentPres**(1/5.257)-1)*TempinKelvin)/.0065

def MetersToFeet(meters):
	return meters*3.28084

def deltaAltitude():
	return MetersToFeet(HypsometricFormula(baro.PRES, baro.TEMP)) - Offset

def LEDAboveBelow(Altitude):
	if Altitude < 0:
		led.setColor('Red')
	else:
		led.setColor('Green')

Offset = MetersToFeet(HypsometricFormula(baro.PRES, baro.TEMP))
	
print "LED is red if negative feet, green if positive"

i=0
while(i<60):
	baro.refreshPressure()
	time.sleep(0.01) # Waiting for pressure data ready 10ms
	baro.readPressure()

	baro.refreshTemperature()
	time.sleep(0.01) # Waiting for temperature data ready 10ms
	baro.readTemperature()

	baro.calculatePressureAndTemperature()
	print "Initial Altitude:"
	print Offset
	print "ALTITUDE: "
	print MetersToFeet(HypsometricFormula(baro.PRES, baro.TEMP))
	#print deltaAltitude()

	LEDAboveBelow(deltaAltitude)

	time.sleep(1)
	
	i=i+1
