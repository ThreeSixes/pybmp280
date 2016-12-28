import smbus
import math
import time
from pprint import pprint

class bmp280:
    def __init__(self, i2cBusID = 1, sensAddr = 0x76):
        """
        Bosch BMP280 barometer driver class.
        """
        
        # Get sensor address.
        self.__sensAddr = sensAddr
        
        # Flags for good readings to be used.
        self.__readingsGood = False
        
        # Store uncompensated T and P values.
        self.__ucT = 0
        self.__ucP = 0
        
        # Store computed temperature and pressure
        self.__presPa = None
        self.__tempC = None
        
        # 16-bit register addresses
        self.__regT1          = 0x88 # unsigned 16
        self.__regT2          = 0x8a # signed 16
        self.__regT3          = 0x8c # signed 16
        self.__regP1          = 0x8e # unsigned 16
        self.__regP2          = 0x90 # signed 16
        self.__regP3          = 0x92 # signed 16
        self.__regP4          = 0x94 # signed 16
        self.__regP5          = 0x96 # signed 16
        self.__regP6          = 0x98 # signed 16
        self.__regP7          = 0x9a # signed 16
        self.__regP8          = 0x9c # signed 16
        self.__regP9          = 0x9e # signed 16
        self.__regRes         = 0xa0 # signed 16
        
        # 8-bit register addresses
        self.__regStat        = 0xf3 # Status byte
        self.__regCtl         = 0xf4 # bit 6-8: temp oversampling, 3-5: pressure oversampling, 1-2: mode
        self.__regConfig      = 0xf5 # bit 6-8: standby time, 3-5: filter, 1 spi2enabled
        self.__regPMsb        = 0xf7 # 1/3 signed long
        self.__regPLsb        = 0xf8 # 1/3 signed long
        self.__regPXlsb       = 0xf9 # 1/3 signed long
        self.__regTMsb        = 0xfa # 1/3 signed long
        self.__regTLsb        = 0xfb # 1/3 signed long
        self.__regTXlsb       = 0xfc # 1/3 signed long
        self.__regReset       = 0xe0 # Reset command.
        self.__regChipID      = 0xd0 # Chip ID
        
        # Status register bits.
        self.__statMeasuring  = 0x08
        self.__statIMUpdate   = 0x01
        
        # Power modes
        self.modeSleep        = 0b00000000
        self.modeForcedA      = 0b00000001
        self.modeForcedB      = 0b00000010
        self.modeNormal       = 0b00000011
        
        # Temperature oversampling settings
        self.osTSkip          = 0b00000000
        self.osT1             = 0b00100000
        self.osT2             = 0b01000000
        self.osT4             = 0b01100000
        self.osT8             = 0b10000000
        self.osT16            = 0b10100000
        self.osT16A           = 0b11000000
        self.osT16B           = 0b11100000
        
        # Pressure oversampling settings
        self.osPSkip          = 0b00000000
        self.osP1             = 0b00000100
        self.osP2             = 0b00001000
        self.osP4             = 0b00001100
        self.osP8             = 0b00010000
        self.osP16            = 0b00010100
        self.osP16A           = 0b00011000
        self.osP16B           = 0b00011100
        
        # Filter coefficients - the datasheet is unclear on these so I am just guessing.
        self.filtOff          = 0b00000000
        self.filt2            = 0b00000100
        self.filt4            = 0b00001000
        self.filt8            = 0b00010000
        self.filt16           = 0b00010100
        
        # Standby time settings
        self.tSb0t5           = 0b00000000
        self.tSb62t5          = 0b00100000
        self.tSb125           = 0b01000000
        self.tSb250           = 0b01100000
        self.tSb500           = 0b10000000
        self.tSb1000          = 0b10100000
        self.tSb2000          = 0b11000000
        self.tSb4000          = 0b11100000
        
        # Chip ID
        self.__chpID = 0
        
        # Try to initialize the i2c bus..
        try:
            # Create the bus address.
            self.__i2cBus = smbus.SMBus(i2cBusID)
        
        except Exception as e:
            # Pass the exception up the stack.
            raise e
        
        # Snag the chip ID.
        try:
            # Get the Chip ID.
            self.__i2cBus.write_byte(sensAddr, self.__regChipID)
            self.__chpID = self.__i2cBus.read_byte(sensAddr)
        
        except Exception as e:
            # Pass the exception up the stack.
            raise e
        
        if self.__chpID <> 0x58:
            # Raise a runtime error.
            raise RuntimeError("Unexpected chip ID for Bosch BMP280: %s" %hex(self.__chpID))
        
        # Get the sensor's coefficients.
        try:
            # Read coefficients...
            coeffBytes = self.__i2cBus.read_i2c_block_data(self.__sensAddr, self.__regT1, 24)
            
            # Build coefficient values...
            self.__t1 = ((coeffBytes[1] << 8) | coeffBytes[0])
            self.__t2 = self.__to16Signed((coeffBytes[3] << 8) | coeffBytes[2])
            self.__t3 = self.__to16Signed((coeffBytes[5] << 8) | coeffBytes[4])
            self.__p1 = ((coeffBytes[7] << 8) | coeffBytes[6])
            self.__p2 = self.__to16Signed((coeffBytes[9] << 8) | coeffBytes[8])
            self.__p3 = self.__to16Signed((coeffBytes[11] << 8) | coeffBytes[10])
            self.__p4 = self.__to16Signed((coeffBytes[13] << 8) | coeffBytes[12])
            self.__p5 = self.__to16Signed((coeffBytes[15] << 8) | coeffBytes[14])
            self.__p6 = self.__to16Signed((coeffBytes[17] << 8) | coeffBytes[16])
            self.__p7 = self.__to16Signed((coeffBytes[19] << 8) | coeffBytes[18])
            self.__p8 = self.__to16Signed((coeffBytes[21] << 8) | coeffBytes[20])
            self.__p9 = self.__to16Signed((coeffBytes[23] << 8) | coeffBytes[22])
        
        except Exception as e:
            # Pass the exception up the stack.
            raise e
    
    def __to16Signed(self, unsignedNum):
        """
        Take an unsigned 16 bit number and convert it to a signed 16 bit number using two's complement.
        """
        # Set return.
        retVal = unsignedNum
        
        # If the sign bit is set...
        if (unsignedNum & 0x8000) > 0:
            # Flip the sign.
            retVal = retVal - 65536
        
        # Return the return value, because it must be returned to be useful - therefore we return it here.
        return retVal
    
    def readSensor(self):
        """
        Read sensor and compute temp + pressure...
        """
        
        try:
            # We should wait for a reading.
            waitForReading = True
            
            # How many times have we tried to get a reading?
            readingAttempts = 0
            
            # Wait until we have good data.
            while waitForReading:
                
                # If we aren't measuring or loading the register...
                if self.__getStat() > 0:
                    # Increment attempts.
                    readingAttempts += 1
                    
                    # Wait for 1/10th of a second.
                    time.sleep(0.1)
                    
                    # If we don't see a reading for 2 seconds.
                    if readingAttempts >= 20:
                        waitForReading = False
                        self.__readingsGood = False
                
                else:
                    # Get the datas.
                    data = self.__i2cBus.read_i2c_block_data(self.__sensAddr, self.__regPMsb, 6)
                    
                    # Build our 20-byte raw pressure and temperature data sequence.
                    pBytes = ((data[0] << 12) | (data[1] << 4) | (data[2]) >> 4)
                    tBytes = ((data[3] << 12) | (data[4] << 4) | (data[5]) >> 4)
                    
                    # Build a big number with uncompensated pressure and temperature.
                    self.__ucP = int(pBytes)
                    self.__ucT = int(tBytes)
                    
                    print("UCP: %s" %self.__ucP)
                    print("UCT: %s" %self.__ucT)
                    
                    # We don't have to wait.
                    waitForReading = False
                    
                    # Compute temperature using the 32-bit formula from the data sheet (roughly)
                    temp1 = ((((self.__ucT >> 3) - (self.__t1 << 1))) * (self.__t2)) >> 11
                    temp2 = (((((self.__ucT >> 4) - (self.__t1)) * ((self.__ucT >> 4) - (self.__t1))) >> 12) * (self.__t3)) >> 14
                    tFine = temp1 + temp2
                    self.__tempC = float((tFine * 5 + 128) >> 8) * 0.01
                    
                    # Compute pressure using the 32-bit formula from the data sheet (roughly)
                    press1 = (tFine >> 1) - 64000
                    press2 = (((press1 >> 2) * (press1 >> 2)) >> 11 ) * self.__p6
                    press2 = press2 + ((press1 * self.__p5) << 1)
                    press2 = (press2 >> 2)+(self.__p4 << 16)
                    press1 = (((self.__p3 * (((press1 >> 2) * (press1 >> 2)) >> 13 )) >> 3) + (((self.__p2) * press1) >> 1)) >> 18
                    press1 = (((32768 + press1) * (self.__p1)) >> 15)
                    
                    if press1 == 0:
                        self.__pressPa = None
                    
                    else:        
                        self.__pressPa = (((1048576 - self.__ucP) - (press2 >> 12))) * 3125
                        
                        if self.__pressPa < 0x80000000:
                            self.__pressPa = (self.__pressPa << 1) / press1
                            
                        else:
                            self.__pressPa = (self.__pressPa / press1) * 2
                        
                        press1 = (self.__p9 * (((self.__pressPa >> 3) * (self.__pressPa >> 3)) >> 13)) >> 12
                        press2 = ((self.__pressPa >> 2) * self.__p8) >> 13
                        self.__pressPa = self.__pressPa + ((press1 + press2 + self.__p7) >> 4)
                    
                    # Flag our readings as good.
                    self.__readingsGood = True
                    
        
        except Exception as e:
            raise e
        
        return
        
    def __getStat(self):
        """
        Get the sensor status byte.
        """
        
        retVal = None
        
        try:
            # Read status byte.
            self.__i2cBus.write_byte(self.__sensAddr, self.__regStat)
            retVal = self.__i2cBus.read_byte(self.__sensAddr)
        
        except:
            raise
        
        return retVal
    
    @property
    def chipID(self):
        """
        Chip ID reported by sensor.
        """
        
        return self.__chpID
    
    @property
    def temperature(self):
        """
        Temperature in degrees C. Returns None when we don't have a good reading.
        """
        retVal = None
        
        if self.__readingsGood == True:
            retVal = round(self.__tempC, 2)
        
        return retVal
    
    @property
    def pressure(self):
        """
        Pressure in Pa. Returns None when we don't have a good reading.
        """
        
        retVal = None
        
        if self.__readingsGood == True:
            retVal = round(self.__pressPa, 2)
        
        return retVal
    
    def getStatRaw(self):
        """
        Get raw status byte. Abstracted to allow overrides.
        """
        
        return self.__getStat()
    
    def getStat(self):
        """
        Get sensor status bytes.
        [0] = im_update
        [1] = measuring
        """
        
        retVal = [None, None]
        
        try:
            # Get raw bytes.
            sensBytes = self.__getStat()
            
            # Check both bits to see if we have a true for either.
            if (sensBytes & self.__statIMUpdate) > 0:
                retVal[0] = True
            else:
                retVal[0] = False
            
            if (sensBytes & self.__statMeasuring) > 0:
                retVal[1] = True
            else:
                retVal[1] = False
        
        except:
            raise
        
        return retVal
    
    def resetSensor(self):
        """
        Reset sensor.
        """
        
        # Write the byte to do the thing.
        self.__i2cBus.write_byte(self.__sensAddr, self.__regReset)
        
        return
    
    def setMode(self, config, meas):
        """
        Set the measurement parameters given config and measurement parameters.
        """
        
        try:
            # Try to write config byte.
            self.__i2cBus.write_byte_data(self.__sensAddr, self.__regConfig, config)
            
            # Try to write measurement control byte.
            self.__i2cBus.write_byte_data(self.__sensAddr, self.__regCtl, meas)
        
        except:
            raise
        
        return


class bmp280Wrapper(bmp280):
    
    @property    
    def sensorMeta(self):
        """
        Dictionary containg sensor metadata.
        """
        
        return {
            'sensor': "BMP280",
            'type': "Barometer and temperature sensor",
            'tempUnit': 'c',
            'pressUnit': 'Pa'
        }
    
    def getStat(self):
        """
        Get the sensor's status.
        """
        
        retVal = {'imUpdate': None, 'measuring': None}
        
        try:
            # Get raw bytes.
            sensBytes = self._bmp280__getStat()
            
            # Check both bits to see if we have a true for either.
            if (sensBytes & self._bmp280__statIMUpdate) > 0:
                retVal['imUpdate'] = True
            else:
                retVal['imUpdate'] = False
            
            if (sensBytes & self._bmp280__statMeasuring) > 0:
                retVal['measuring'] = True
            else:
                retVal['measuring'] = False
        
        except:
            raise
        
        return retVal
    
    def getTemperature(self):
        """
        Get temperature data from the sensor with metadata.
        """
        
        retVal = {'temp': self.__tempC, 'unit': 'c'}
        
        return retVal
    
    def getPressure(self):
        """
        Get pressure from the sensor with metadata.
        """
        
        retVal = {'pressure': self.__presPa, 'unit': 'Pa'}
        
        return retVal
    
    def getAlt(self, pressure):
        """
        Get altitude given a pressure in pascals.
        """
        
        retVal = 0
        
        alt = 44330 * (1 - (pressure / 101325) * (1/5.255))
        
        # This appears to be in mm, so convert to m.
        alt = alt * 0.001
        
        retVal = {'altitude': round(alt, 2), 'unit': 'm', 'meta': 'experimental'}
        
        return retVal
