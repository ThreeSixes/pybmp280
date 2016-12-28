#!/usr/bin/python

import traceback
import bmp280
import time
from pprint import pprint

print("Init BMP280...")

try:
    sensW = bmp280.bmp280Wrapper()
except:
    print("Failed to init sensor: %s" %traceback.format_exc())
    quit

print("Found BMP280 (%s)." %hex(sensW.chipID))

# Restart sensor.
sensW.resetSensor()

# Set the sensor up in hand-held device low power mode - the first
# use case in table 15 in the spec sheet.

# Our configuration byte contains standby time, filter, and SPI enable.
bmp280Config = sensW.tSb62t5 | sensW.filt4
# Our measurement byte contains temperature + pressure oversampling and mode.
bmp280Meas = sensW.osP16 | sensW.osT2 | sensW.modeNormal

# Set sensor mode.
sensW.setMode(config = bmp280Config, meas = bmp280Meas)

while True:
    # Take a reading.
    sensW.readSensor()
    
    # Print pressure
    print("Pressure   : %s Pa" %sensW.pressure)
    
    # Print temperature
    print("Temperature: %s C" %sensW.temperature)
    
    # Wait a second.
    time.sleep(1)