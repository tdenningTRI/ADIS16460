# ADIS16460
Make sure SPI is enabled in Raspberry Pi Config. Must enable the pigpio daemon with the command prompt command: sudo pigpiod

Initialize by calling IMU(sampleRate, dataReadyPin, debug)
call update() as frequently as possible to update member values: xGyro, yGyro, zGyro, xAccel, yAccel, zAccel, intTemp, lastTime.
During initialization, sets startTime member to time since the epoch in nanoseconds. 
The dataready pin is pulled high when data is available as frequently as set in sampleRate. lastTime updates the time of the last sample similarly to startTime initialization. 

