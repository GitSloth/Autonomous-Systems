#import upip
#upip.install('micropython-vl53l0x')


import time
from machine import Pin, I2C
from vl53l0x import VL53L0X

# Define the I2C pins
sda_pin = Pin(0)  # Change this to your specific SDA pin
scl_pin = Pin(1)  # Change this to your specific SCL pin

# Initialize the I2C bus
i2c = I2C(0, sda=sda_pin, scl=scl_pin, freq=400000)

# Scan for I2C devices
devices = i2c.scan()
print("I2C devices found:", devices)

# Create a VL53L0X object
tof = VL53L0X(i2c)

# Main loop
while True:
    # Start ranging
    tof.start()
    
    # Read distance
    distance_mm = tof.read()
    
    # Print distance
    print("Distance: {} mm".format(distance_mm))
    
    # Stop ranging
    tof.stop()
    
    # Delay before next measurement
    time.sleep(0.1)
