This is a new MicroPython driver for the BME680 temperature, humidity, pressure and VOC sensor. The driver was written from scratch in order to try overcoming the issues I encountered with existing drivers, after testing and reading the code of existing drivers and other C-coded drivers available in the wild.

Since this is a complete rewrite, I prefixed the driver name with "az" to make sure that it is easy to distinguish from the many forks for MicroPython / CircuitPython I found of variations and fixes of the same driver.

# API

## Creating the BME680 object

Just pass an i2c object as first and only argument to the constructor.

    i2c = SoftI2C(scl=Pin(5),sda=Pin(16))
    bme = BME680(i2c)

The i2c object needs to support the following methods:

    i2c.readfrom_mem()
    i2c.writeto_mem()

Configuration of parameters like oversampling, IIR filter and what reading
to take are all performed via the other APIs and not via initialization.

## Obtaining measures

This driver is conceived to take measures in two step. The first call
is used in order to actually set the BME680 in "forced" mode and obtain
an analogical reading of the selected channels (temperature, pressure,
humidity, gas).

    bme.measure()   # Obtain temp, humidity, pressure from the chip.

By default, gas reading is not performed. To read the VOC/gas resistance
value, in ohm, the chip requires to heat the reacting surface, wait some
time and so forth. This process is both slow and expensive in terms of
consumed currnet, so if you want this reading, use:

    bme.measure(gas=True)

Then the conversion from the raw data to the actual measure is obtained
using the following set of APIs:

    bme.temperature()       # Temperature in celsius
    bme.pressure()          # Pressure in hPascal
    bme.humidity()          # Relative humidity
    bme.gas()               # Gas sensor resistance in ohms

All the above calls return just the value, but `.gas()` which returns
a dictionary like this:

    {'ohms': 225257, 'valid': True}

This is required because sometimes the chip will tell us that the reading
is invalid, as there were issues with heating the VOC-sensitive surface and
so forth.

You can also obtain the estimated location altitude in meters or feet:

    bme.measure()
    p = bme.pressure()
    bme.altitude(p)
    
    # Or, for a better approximation, provide the
    # pressure at sea level.
    bme.altitude(p, sea_level_p = 1013.25)

## Changing oversampling and filter

For more precise readings, the BME680 performs optional oversampling of
temperature, pressure and humidity values. You can select different
oversampling values for each of the channels.

Valid oversampling values are: 0, 1, 2, 4, 8, 16. A value of zero means
to skip the measure completely. Greater oversampling values correspond
to greater measure delay but lower noise.

For example the following measure will be performed selecting only
the temperature, with an oversampling value of 8.

    bme.measure(gas=False,p_os=0,t_os=8,h_os=0)
    print(bme.temperature())

It is also possible to select and change settings for the IIR (infinite
impluse response) filter. The IIR filter is used in order to smooth
the readings of temperature and pressure, that are subject to strong noise.
A value of 0 totally switches off the IIR filter.

Valid filter coefficient settings are: 0, 1, 3, 7, 15, 31, 63, 127.
A higher filter coefficient means more aggressive filtering.

For instance if I want unfiltered pressure data, I'll use:

    bme.measure(gas=False,p_os=1,t_os=0,h_os=0,iir_filter=0)
    print(bme.pressure())

## Changing VOC reading parameters

The chip is already kinda magic enough, thanks to Bosch(TM) providing
only a closed source library for transforming the raw gas ohm readings
to an air quality index. Moreover it is possible to select the heating
temperature of the gas detecting surface and the milliseconds we will
wait in order to read the value.

I believe it's better to stick with the parameters everybody seems to
use:

    320 degrees, 150 milliseconds

However if you really want to play with this, use:

    bme.measure(gas=True,gas_temp=350,gas_ms=200)

Temperatures range from 200 to 400 degrees.
Heating-on times up to a bit more than 4000 milliseconds.
See the `set_gas_heater()` method in the code for details and
make sure to check the datasheet.

## Examples

The `example.py` file is just a very simple example that shows how to setup the
i2c object and use it to create the BMP680 object. It will print
all the available readings using the default oversampling and filtering.

The `door_detector.py` example shows how to get unfiltered data with
less delay, in order to detect if somebody is opening/closing a door
in your room.
