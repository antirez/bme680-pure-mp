from azbme680 import *
from machine import SoftI2C, Pin
import time

i2c = SoftI2C(scl=Pin(5),sda=Pin(16))
bme = BME680(i2c)

while True: 
    bme.measure(gas=True)
    print("* T,H,P,ALT,gas:",bme.temperature(), bme.humidity(), bme.pressure(), bme.altitude(bme.pressure()), bme.gas())
    for i in range(10):
        bme.measure(gas=False)
        print("- T,H,P:",bme.temperature(), bme.humidity(), bme.pressure())
        time.sleep(1)
