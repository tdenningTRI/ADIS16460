import pigpio
import time




class IMU:
    def __init__(self, sampleRate, dataReadyPin = 25, taps = 4, debug = False):
        """
        Initializes the SPI interface. Put IMU.update() in a while loop to update the member variable values. Important member variables are as follows:
        startTime
        lastTime
        xGyro
        yGyro
        zGyro
        xAccel
        yAccel
        zAccel
        intTemp

        Parameters:
        sampleRate int -> in Hz max Sample Rate is 2048. Note: You can put any sample rate you want, but it will be rounded to nearest integer division of 2048. ex 2048/2 = 1024hz, 2048/3 = 682.67hz, etc etc. 
        dataReadyPin int -> the data ready pin toggles high at the given sample rate. 
        debug -> when set to True, provides "helpful" debugging info
        
        Returns
        """
        self.debug = debug
        self.ready = False
        self.sampleRate = sampleRate
        self.startTime = self.get_time()
        self.lastTime = self.get_time()
        self.imu_init(sampleRate, taps)
        self.set_accel_scale_factor()
        self.set_gyro_scale_factor()
        self.dr_init(dataReadyPin)
        self.filter_init(taps)
        self.read()
        

    def imu_init(self, rate, taps):
        '''
        Creates the SPI connection. Called when class instantiated. opens SPI and writes the Dec rate(sample rate) to the register
        
        Parameters: rate -> defined in class instantiation
        
        Returns: True if succesfully connected. False otherwise. In Debug Will print recognized connected FTDI devices
        '''
        
        try:
            self.pi = pigpio.pi()
            self.spi = self.pi.spi_open(0,1000000,3)
            self.set_dec_rate(rate)
            return True
        
        except Exception as e:
            print(e)
            return False

    def set_dec_rate(self, rate):
        """
        Called in the imu_init function. This function calculates the decimation factor based on the user's desired sample rate. The default sample rate on
        the ADIS16460 is 2048 samples per second. Sample rates greater than 2048 will default to 2048. 
        Decimation filters necessarily divide sample rate by integers only. As a result, this function rounds the desired sample rate to the nearest possible sample rate.
        
        Parameters: rate -> same as above

        Returns: none"""
        self.dec = int(2048/rate) - 1
        if self.dec < 0:
            self.dec = 0
        write = 0xB600 + self.dec
        w = write.to_bytes(2, byteorder = 'big', signed = False)
        self.pi.spi_write(self.spi,w)
        write = 0xB700
        w = write.to_bytes(2, byteorder = 'big', signed = False)
        self.pi.spi_write(self.spi,w)
    
    def filter_init(self, taps):
        '''
        Sets the number of taps for the Bartlett Window FIR filter. See data sheet for frequency response. Must be an integer from 0-7 inclusive Numbers outside that range return false
        
        Parameters:
        taps -> 3 bit integer
        
        returns: False if invalid number passed'''

        if taps < 0 or taps > 7:
            return False
        write = 0xB800 + taps
        w = write.to_bytes(2, byteorder = 'big', signed = False)
        self.pi.spi_write(self.spi,w)

    def dr_init(self, pin):
        '''
        initializes the data ready(dr) interrupt pin. The dr pin transitions to high when data is available on the output registers. sets self.ready to true in order for it to be checked by the update method
        
        Parameters:
        GPIO input pin 
        
        returns : None
        '''
        def ready(gpio, level, tick):
            self.ready = True
            #print("Ready")
        self.dr = self.pi.callback(pin, pigpio.RISING_EDGE, ready)
    
    
    def update(self):
        """
        Calls read function if DR pin pulsed high since last call of this function. 
        Currently updates the member variables of the IMU object, but could instead return those values as a list if needed.
        
        Parameters: int freq of update in hz.

        Returns: None
        """
       
        if(self.ready):
            self.read()
            self.ready = False
            

    def read(self):
        """
        Sets self.xGyro, self.yGyro, self.zGyro, self.xAccel, self.yAccel, self.zAccel, self.intTemp. Called by the update function when DR pin indicates data available

        parameters: None
        
        Returns: None
        """
        
        diag = self.read_from_register(0x0200)
        rawXGyro = self.read_from_register(0x0600) + self.read_from_register(0x0400)
        rawYGyro = self.read_from_register(0x0A00) + self.read_from_register(0x0800)
        rawZGyro = self.read_from_register(0x0E00) + self.read_from_register(0x0C00)
        rawXAccel = self.read_from_register(0x1200) + self.read_from_register(0x1000)
        rawYAccel = self.read_from_register(0x1600) + self.read_from_register(0x1400)
        rawZAccel = self.read_from_register(0x1A00) + self.read_from_register(0x1800)
        rawTemp = self.read_from_register(0x1E00)
        self.lastTime = self.get_time()
        
        if self.debug: #SHOULD PRINT 404C. Tests to see if reading properly
            print(self.read_from_register(0x5600).hex())
        self.xGyro = self.real_gyro(self.twos_to_dec(rawXGyro))
        self.yGyro = self.real_gyro(self.twos_to_dec(rawYGyro))
        self.zGyro = self.real_gyro(self.twos_to_dec(rawZGyro))
        self.xAccel = self.real_accel(self.twos_to_dec(rawXAccel))
        self.yAccel = self.real_accel(self.twos_to_dec(rawYAccel))
        self.zAccel = self.real_accel(self.twos_to_dec(rawZAccel))
        self.intTemp = self.twos_to_dec(rawTemp)*0.05 +25
        if self.debug:
            print(f"X Gyro Reading: {self.xGyro:.3f} Y Gyro Reading: {self.yGyro:.3f} Z Gyro Reading: {self.zGyro:.3f} X Accel Reading: {self.xAccel:.3f} Y Accel Reading: {self.yAccel:.3f} Z Accel Reading: {self.zAccel:.3f}  Internal Temp: {self.intTemp}         ",
            end = '\r')
    
    def read_from_register(self,reg):
        '''
        reads from the given register
        Parameters: reg as a 2 byte unsigned hex integer
        returns: received data in the form of a 2 byte, bytearray
        '''
        
        
        w = reg.to_bytes(2, byteorder = 'big', signed = False)
        self.pi.spi_write(self.spi,w)
        (num, received) = self.pi.spi_read(self.spi,2)
    
        return received
        
    
    def twos_to_dec(self, sample):
        """
        Samples come in the form of 16 or 32 bit twos complement data. Converts to equivalent decimal

        Parameters:
        byte: sample in 16 or 32 bit hex

        Returns:
        int decimal equivalent
        """
        return int.from_bytes(sample, byteorder = 'big', signed = True)


    def real_accel(self, val):
        """
        Scales the decimal value acquired in twos_to_dec to the real acceleration in units depending on the scale factor

        Parameters:
        int: decimal converted from twos_to_dec function

        Returns:
        float: real acceleration with units corresponding to the scale factor. (mm/s^2)
        """
        return val*self.accelScaleFactor
    
    def real_gyro(self, val):
        """
        Scales the decimal value acquired in twos_to_dec to the real gyro in units depending on the scale factor

        Parameters:
        int: decimal converted from twos_to_dec function

        Returns:
        float: real gyro readin with units corresponding to the scale factor. (°/s^2)
        """
        return val*self.gyroScaleFactor
    
    def get_time(self):
        """
        returns current system time

        Parameters:
        none

        Returns:
        int: system time
        """
        return time.monotonic_ns()
    
    


    def set_accel_scale_factor(self):

        """
        Sets the acceleration scale factor according to the datasheet formula. Units are mm/s^2. Data from the unit comes in mg. set g to 1 if you want units of mg for calibration purposes
        
        Parameters: None
        
        Returns: None
        """
        g = 9.80665
        # g=1
        self.accelScaleFactor = 0.25/(2**(16))*g/1000

    def set_gyro_scale_factor(self):
        """
        Sets the gyro scale factor according to the datasheet formula. Units are °/s.
        
        Parameters: None
        
        Returns: None
        """
        self.gyroScaleFactor = 0.005/(2**(16))



if __name__ == "__main__":
    o = IMU(sampleRate=30, taps = 4, debug = True)
    ns = 0
    
    while(ns/1000000000 < 5):
        
        o.update()
        ns = o.lastTime - o.startTime
        # time.sleep(1)
        # print(ns/1000000000)
    o.pi.spi_close(o.spi)
    # print(o.count)
