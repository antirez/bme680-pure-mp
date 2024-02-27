from azbme680 import *
from machine import SoftI2C, Pin
import time
i2c = SoftI2C(scl=Pin(5),sda=Pin(16))
bme = BME680(i2c)

print("!!! Close or open a door vigorously. This should detect it")
readings = []
avg = None
while True: 
    bme.measure(gas=False,p_os=1,t_os=0,h_os=0,iir_filter=0)
    this_p = bme.pressure()

    # Each 50 readings, set the new average pressure.
    readings.append(this_p)
    if len(readings) == 50:
        avg = sum(readings)/len(readings)
        readings = []
        print("avg:",avg)

    # If we have a solid average, check if the current reading
    # is too far from the average. Somebody closed/opened the door
    # (or a similiar sudden change in pressure happened).
    if avg != None:
        delta = this_p-avg
        if delta < 0: delta = -delta
        if delta > 0.4:
            if this_p > avg:
                print("Door opened, p=",this_p)
            else:
                print("Door closed, p=",this_p)
