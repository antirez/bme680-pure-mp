# BMA680 MicroPython driver.
#
# Copyright (c) 2024 Salvatore Sanfilippo.
# This code is released under the MIT License.

import time # For milliseconds sleep.
import math # For pow() in pressure to altitude conversion.

# Register addresses
BME680_REG_CHIPID = const(0xD0)
BME680_REG_SOFTRESET = const(0xE0)
BME680_REG_CTRL_GAS_1 = const(0x70)
BME680_REG_CTRL_GAS_0 = const(0x71)
BME680_REG_CTRL_HUM = const(0x72)
BME680_REG_CTRL_MEAS = const(0x74)
BME680_REG_CONFIG = const(0x75)
BME680_REG_MEAS_STATUS = const(0x1D)
BME680_REG_COEFF_ADDR1 = const(0x89)
BME680_REG_COEFF_ADDR2 = const(0xE1)
BME680_REG_RES_HEAT_0 = const(0x5A)
BME680_REG_GAS_WAIT_0 = const(0x64)

# Register values constants
BME680_CHIPID = const(0x61)                         # BME680_REG_CHIPID
BME680_OVERSAMPLING = (0, 1, 2, 4, 8, 16)           # BME680_REG_CTRL_MEAS
BME680_IIR_FILTER = (0, 1, 3, 7, 15, 31, 63, 127)   # BME680_REG_CONFIG

# gas_range_r selects a set of constants, used in order to convert
# the raw ADC value to resistance in ohms.
# See Table 16 in the BMA290 datasheet.
CONST_ARRAY1_INT = (2147483647, 2147483647, 2147483647, 2147483647,
                    2147483647, 2126008810, 2147483647, 2130303777,
                    2147483647, 2147483647, 2143188679, 2136746228,
                    2147483647, 2126008810, 2147483647, 2147483647)

CONST_ARRAY2_INT = (4096000000, 2048000000, 1024000000, 512000000,
                     255744255,  127110228,   64000000,  32258064,
                      16016016,    8000000,    4000000,   2000000,
                       1000000,     500000,     250000,    125000)

class BME680:
    def __init__(self, i2c):
        self.i2c = i2c
        self.i2c_address = 0x77 # Default i2c address for the BMP680.

        # Resed and device ID check.
        self.reset()
        chip_id = self.getreg(BME680_REG_CHIPID)
        if chip_id != BME680_CHIPID:
            raise RuntimeError('Failed to find BME680! Chip ID 0x%x' % chip_id)

        self.read_calibration_params() # These never change, chip specific.
        self.t_fine = None

    # Soft reset.
    def reset(self):
        self.setreg(BME680_REG_SOFTRESET, 0xB6)
        time.sleep_ms(5)

    # Return temperature in Celsius. Must be called after measure().
    def temperature(self):
        calc_temp = (((self.t_fine * 5) + 128) / 256)
        return calc_temp / 100

    # Return temperature in hPascals. Must be called after measure().
    def pressure(self):
        adc_pres = (self.getreg(0x1f)<<12) | (self.getreg(0x20)<<4) | (self.getreg(0x21)>>4)
        var1 = (self.t_fine >> 1) - 64000
        var2 = (((var1 >> 2) ** 2) >> 11) * (self.par_p6 >> 2)
        var2 = var2 + ((var1 * self.par_p5) << 2)
        var2 = (var2 >> 2) + (self.par_p4 << 16)
        var1 = (((((var1 >> 2) ** 2) >> 13) * (self.par_p3 << 5)) >> 3) + ((self.par_p2 * var1) >> 1)
        var1 = var1 >> 18
        var1 = ((32768 + var1) * self.par_p1) >> 15
        press_comp = 1048576 - adc_pres
        press_comp = (press_comp - (var2 >> 12)) * 3125
        if press_comp >= (1<<30):
            press_comp = (press_comp // var1) << 1
        else:
            press_comp = ((press_comp << 1) // var1)
        var1 = (self.par_p9 * (((press_comp >> 3) ** 2) >> 13)) >> 12
        var2 = ((press_comp >> 2) * self.par_p8) >> 13
        var3 = (((press_comp >> 8) ** 3) * self.par_p10) >> 17
        press_comp += ((var1 + var2 + var3 + (self.par_p7 << 7)) >> 4)
        return press_comp/100 # Convert to hPascals.

    # Return relative humidity. Must be called after measure().
    def humidity(self):
        hum_adc = (self.getreg(0x25)<<8)|self.getreg(0x26)
        temp_scaled = ((self.t_fine * 5) + 128) >> 8
        var1 = hum_adc - (self.par_h1 << 4) - (((temp_scaled * self.par_h3) // 100) >> 1)
        var2 = (self.par_h2 * (((temp_scaled *
                self.par_h4) // 100) +
                (((temp_scaled * ((temp_scaled * self.par_h5) //
                100)) >> 6) // 100) + 16384)) >> 10
        var3 = var1 * var2
        var4 = ((self.par_h6 << 7) + ((temp_scaled * self.par_h7) // 100)) >> 4
        var5 = ((var3 >> 14) ** 2) >> 10
        var6 = (var4 * var5) >> 1
        calc_hum = (((var3 + var6) >> 10) * 1000) >> 12
        calc_hum /= 1000  # Convert to percentage.
        calc_hum = min(max(calc_hum,0),100)
        return calc_hum

    # Return altitude in meters or feet. Use sea level pressure
    # as reference.
    def altitude(self,pressure,unit="meters",*,sea_level_p=1013.25):
        # Constant depends the selected unit. Meters or feets.
        if unit == "meters":
            c = 44300
        elif unit == "feet":
            c = 145366.45
        else:
            raise ValueError("Invalid unit for altitude")
        return c * (1.0 - math.pow(pressure / sea_level_p, 0.190284))

    # Return VOC sensor resistance in ohms. This value is not absolute
    # and is not the quality index that the BME680 datasheet talks about:
    # that AQI value is only computed in software by their proprietary
    # library. Here you are dealing with the raw data.
    def gas(self):
        gas_status = self.getreg(0x2b)
        gas_reading_valid = gas_status & (1<<5)
        gas_heat_stab = gas_status & (1<<4)
        adc_gas = (self.getreg(0x2a) << 2) | self.getreg(0x2b) >> 6
        gas_range = self.getreg(0x2b) & 0xf
        var1 = ((1340 + (5 * self.range_switching_error)) * (CONST_ARRAY1_INT[gas_range])) >> 16
        var2 = (adc_gas << 15) - 16777216 + var1
        calc_gas_res = (((CONST_ARRAY2_INT[gas_range] * var1) >> 9) + (var2 >> 1)) / var2
        ohms = int(calc_gas_res)
        valid = gas_reading_valid != 0 and gas_heat_stab != 0
        return {"ohms": ohms, "valid": valid}

    # Eanble/disable gas reading for the next reading. Gas reading
    # is slower, requires more current (because the reactive surface
    # must be heated at high temperature), so when it's not needed it's
    # better to skip it.
    def set_gas_enabled(self,gas=True):
        gas = (int(gas) & 1) << 4 # Gas on/off bit is bit 4.
        ctrl_gas_0 = self.getreg(BME680_REG_CTRL_GAS_0)
        ctrl_gas_0 = (ctrl_gas_0 & 0b11101111) | gas
        self.setreg(BME680_REG_CTRL_GAS_0, ctrl_gas_0)

    # Set the gas heater temperature and heater-on time in milliseconds.
    def set_gas_heater(self,*,temp=320,ms=150):
        # Select temperature.
        amb_temp = 20
        res_heat_range = (self.getreg(0x02)>>4)&3
        res_heat_val = self.int8(self.getreg(0x00))

        var1 = ((amb_temp*self.par_g3)//10) << 8
        var2 = (self.par_g1+784) * (((((self.par_g2+154009)*temp*5)//100)+3276800)//10)
        var3 = var1 + (var2>>1)
        var4 = var3 // (res_heat_range+4)
        var5 = (131*res_heat_val) + 65536
        res_heat_x100 = ((var4//var5)-250)*34
        res_heat_x = (res_heat_x100+50)//100
        # 320 degrees should correspond to a value of 0x73
        self.setreg(BME680_REG_RES_HEAT_0, res_heat_x)

        # Time selection: we can use multipliers of 1, 4, 16, 64
        # and a base value between 0 and 63. So the maximum heating
        # time is 63*64 milliseconds (~4 seconds).
        if ms < 0 or ms > 63*64: raise ValueError("Invalid heater-on time")

        if ms < 63:
            mul = 0 # x1 multiplier
        elif ms < 63*4:
            mul = 1 # x4 multiplier
        elif ms < 63*16:
            mul = 2 # x16 multiplier
        else:
            mul = 4 # x64 multiplier
        base_ms = ms//mul

        self.setreg(BME680_REG_GAS_WAIT_0, (mul<<6)|base_ms)

        # We use always the first slot of the heater many configuration
        # slots available, so we have to set the slot ID to 0
        # in the CTRL_GAS_1 sub-field.
        regval = self.getreg(BME680_REG_CTRL_GAS_1)
        self.setreg(BME680_REG_CTRL_GAS_1, regval & 0xF0)

    # Oversampling value to register value.
    def get_oversampling_val(self,os):
        if os not in BME680_OVERSAMPLING:
            raise ValueError("Invalid IIR filter coefficient.")
        return BME680_OVERSAMPLING.index(os)

    # Take a measurement from the sensor. The gas is measured only
    # if gas is True (False by default).
    #
    # p_os, t_os, h_os control the oversampling of pressure, temperature,
    # humidity outputs. More oversampling means more delay but more precision.
    # Valid values are 0, 1, 2, 4, 8, 16.
    # The special value "0" means to skip this measure at all.
    # 
    # iir_filter selects the IIR filter coefficient. A value of 0 disables
    # the filter. The filter is only used for pressure and temperature, because
    # gas/humidity don't change so fast. If you want to detect sudden changes
    # in pressure (door opens/closes), use 0.
    #
    # Valid avalues for filter is: 0, 1, 3, 7, 15, 31, 63, 127.
    def measure(self,gas=False,p_os=4,t_os=8,h_os=2,iir_filter=3,gas_temp=320,gas_ms=150):
        # Default oversampling and filter register values.
        pressure_oversampling = self.get_oversampling_val(p_os)
        temp_oversampling = self.get_oversampling_val(t_os)
        humidity_oversampling = self.get_oversampling_val(h_os)

        if iir_filter not in BME680_IIR_FILTER:
            raise ValueError("Invalid IIR filter coefficient.")
        filter_size = BME680_IIR_FILTER.index(iir_filter)

        # Set filter size and oversampling paramenters for
        # this reading.
        self.setreg(BME680_REG_CONFIG, filter_size << 2)
        self.setreg(BME680_REG_CTRL_MEAS,
                    (temp_oversampling << 5)|
                    (pressure_oversampling << 2))
        self.setreg(BME680_REG_CTRL_HUM, humidity_oversampling)

        if gas:
            # If a gas reading is requested, enable gas, set up heater
            # temperature and heater-on time for profile 0.
            self.set_gas_enabled(gas)
            self.set_gas_heater(temp=gas_temp,ms=gas_ms)

        # Enable forced mode. After the reading the device will return
        # to sleep mode.
        ctrl = self.getreg(BME680_REG_CTRL_MEAS)
        ctrl = (ctrl & 0xFC) | 0x01
        self.setreg(BME680_REG_CTRL_MEAS, ctrl)

        # Wait for the reading to complete.
        new_data = 0
        while new_data == 0:
            new_data = self.getreg(BME680_REG_MEAS_STATUS) & 0x80
            time.sleep_ms(5)

        # Calculate the t_fine value ASAP, which is used not just in the
        # temperature reading, but also in all the other
        # temperature-compensated readings.
        adc_temp = (self.getreg(0x22)<<12) | (self.getreg(0x23)<<4) | (self.getreg(0x24)>>4)
        var1 = (adc_temp >> 3) - (self.par_t1 << 1)
        var2 = (var1 * self.par_t2) >> 11
        var3 = ((((var1 >> 1) * (var1 >> 1)) >> 12) * (self.par_t3 << 4)) >> 14;
        self.t_fine = var2 + var3

    # Convert unsigned representation of i8 to signed.
    def int8(self,val):
        return val if val & (1<<7) == 0 else -256+val

    # Convert unsigned representation of i16 to signed.
    def int16(self,val):
        return val if val & (1<<15) == 0 else -65536+val

    # This method reads the calibration data stored inside the device
    # and populates the calibration attributes of the object.
    def read_calibration_params(self):
        co = self.getreg(BME680_REG_COEFF_ADDR1, 25) + \
             self.getreg(BME680_REG_COEFF_ADDR2, 16)

        # Temperature calibration parameters.
        self.par_t1 = (co[34]<<8) | co[33]
        self.par_t2 = self.int16((co[2]<<8) | co[1])
        self.par_t3 = self.int8(co[3])

        # Pressure calibration parameters.
        self.par_p1 = (co[6]<<8) | co[5]
        self.par_p2 = self.int16((co[8]<<8) | co[7])
        self.par_p3 = self.int8(co[9])
        self.par_p4 = self.int16((co[12]<<8) | co[11])
        self.par_p5 = self.int16((co[14]<<8) | co[13])
        self.par_p6 = self.int8(co[16])
        self.par_p7 = self.int8(co[15])
        self.par_p8 = self.int16((co[20]<<8) | co[19])
        self.par_p9 = self.int16((co[22]<<8) | co[21])
        self.par_p10 = co[23]

        # Humidity calibration parameters.
        self.par_h1 = (co[27] << 4) | (co[26] & 0xf)
        self.par_h2 = (co[25] << 4) | (co[26] >> 4)
        self.par_h3 = self.int8(co[28])
        self.par_h4 = self.int8(co[29])
        self.par_h5 = self.int8(co[30])
        self.par_h6 = co[31]
        self.par_h7 = self.int8(co[32])

        # Gas sensor coration parameters.
        self.par_g1 = self.int8(co[37])
        self.par_g2 = self.int16((co[36]<<8) | co[35])
        self.par_g3 = self.int8(co[38])
        self.range_switching_error = self.getreg(0x04)

    def getreg(self, register, length=None):
        if length != None:
            return self.i2c.readfrom_mem(self.i2c_address,register,length)
        else:
            return self.i2c.readfrom_mem(self.i2c_address,register,1)[0]

    def setreg(self, register, values):
        if isinstance(values,int): values = bytes([values])
        self.i2c.writeto_mem(self.i2c_address, register, values)
