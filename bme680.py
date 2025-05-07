# The MIT License (MIT)
#
# Copyright (c) 2017 ladyada for Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# We have a lot of attributes for this complex sensor.
# pylint: disable=too-many-instance-attributes

"""
`bme680` - BME680 - Temperature, Humidity, Pressure & Gas Sensor
================================================================

MicroPython driver from BME680 air quality sensor, based on Adafruit_bme680

* Author(s): Limor 'Ladyada' Fried of Adafruit
             Jeff Raber (SPI support)
             and many more contributors
"""

import struct
import time
import math
import smbus2


class bme680:
    """
    Driver from BME680 air quality sensor
    """

    CHIPID = 0x61

    REG_CHIPID = 0xD0
    REG_VARIANT = 0xF0
    COEFF_ADDR1 = 0x89
    COEFF_ADDR2 = 0xE1
    RES_HEAT_0 = 0x5A
    GAS_WAIT_0 = 0x64

    REG_SOFTRESET = 0xE0
    REG_CTRL_GAS = 0x71
    REG_CTRL_HUM = 0x72
    REG_STATUS = 0xF3
    CTRL_MEAS = 0x74
    REG_CONFIG = 0x75

    REG_PAGE_SELECT = 0x73
    REG_MEAS_STATUS = 0x1D
    REG_PDATA = 0x1F
    REG_TDATA = 0x22
    REG_HDATA = 0x25

    OS_1X = 1
    OS_2X = 2
    OS_4X = 3
    OS_8X = 4
    OS_16X = 5

    FILT_1 = 1
    FILT_3 = 2
    FILT_7 = 3
    FILT_15 = 4
    FILT_31 = 5
    FILT_63 = 6
    FILT_127 = 7

    RUNGAS = 0x10

    LOOKUP_TABLE_1 = (
        2147483647.0,
        2147483647.0,
        2147483647.0,
        2147483647.0,
        2147483647.0,
        2126008810.0,
        2147483647.0,
        2130303777.0,
        2147483647.0,
        2147483647.0,
        2143188679.0,
        2136746228.0,
        2147483647.0,
        2126008810.0,
        2147483647.0,
        2147483647.0,
    )

    LOOKUP_TABLE_2 = (
        4096000000.0,
        2048000000.0,
        1024000000.0,
        512000000.0,
        255744255.0,
        127110228.0,
        64000000.0,
        32258064.0,
        16016016.0,
        8000000.0,
        4000000.0,
        2000000.0,
        1000000.0,
        500000.0,
        250000.0,
        125000.0,
    )

    @classmethod
    def _bigEndian(cls, bts: bytes):
        """
        Assemble a big-endian unsigned in from unsigned byteBME680_I2Cs.
        """
        ret: int = 0
        for b in bts:
            ret *= 256
            ret += b & 0xFF
        return ret

    def __init__(self):
        """
        Check the BME680 was found, read the coefficients and enable the sensor for continuous
        reads.
        """

        self._write(bme680.REG_SOFTRESET, [0xB6])
        time.sleep(0.1)

        chip_id = self._read_byte(bme680.REG_CHIPID)
        if chip_id != bme680.CHIPID:
            raise RuntimeError("Failed to find BME680! Chip ID 0x%x" % chip_id)

        self.variant = self._read_byte(bme680.REG_VARIANT)

        self._read_calibration()

        # set up heater
        self._write(bme680.RES_HEAT_0, [0x73])
        self._write(bme680.GAS_WAIT_0, [0x65])

        # Pressure in hectoPascals at sea level. Used to calibrate ``altitude``.
        self.sea_level_pressure = 1013.25

        # local copies of raw data from sensor chip
        self._adc_pres = None
        self._adc_temp = None
        self._adc_hum = None
        self._adc_gas = None
        self._gas_range = None
        self._t_fine = None

        self._pressure_oversample = None
        self._temp_oversample = None
        self._humidity_oversample = None
        self._filter_size = None

        # Default oversampling and filter register values,
        # set via the property setters below.
        self.pressure_oversample = bme680.OS_4X
        self.temp_oversample = bme680.OS_8X
        self.humidity_oversample = bme680.OS_2X
        self.filter_size = bme680.FILT_3

    @property
    def pressure_oversample(self):
        """The oversampling for pressure sensor"""
        return self._pressure_oversample

    @pressure_oversample.setter
    def pressure_oversample(self, os_factor):
        if os_factor < bme680.OS_1X or os_factor > bme680.OS_16X:
            raise ValueError("Invalid oversample factor {}.".format(os_factor))
        self._pressure_oversample = os_factor

    @property
    def humidity_oversample(self):
        """The oversampling for humidity sensor"""
        return self._humidity_oversample

    @humidity_oversample.setter
    def humidity_oversample(self, os_factor: int):
        if os_factor < bme680.OS_1X or os_factor > bme680.OS_16X:
            raise ValueError("Invalid oversample factor {}.".format(os_factor))
        self._humidity_oversample = os_factor

    @property
    def temperature_oversample(self):
        """The oversampling for temperature sensor"""
        return self._temp_oversample

    @temperature_oversample.setter
    def temperature_oversample(self, os_factor: int):
        if os_factor < bme680.OS_1X or os_factor > bme680.OS_16X:
            raise ValueError("Invalid oversample factor {}.".format(os_factor))
        self._temp_oversample = os_factor

    @property
    def filter_size(self):
        """The filter size for the built in IIR filter"""
        return self._filter_size

    @filter_size.setter
    def filter_size(self, size: int):
        if size < bme680.FILT_1 or size > bme680.FILT_127:
            raise ValueError("Invalid filter length {}.".format(size))
        self._filter_size = size

    @property
    def temperature(self) -> float:
        """The compensated temperature in degrees celsius."""
        calc_temp = ((self._t_fine * 5) + 128) / 256
        return calc_temp / 100

    @property
    def pressure(self) -> float:
        """The barometric pressure in hectoPascals"""
        var1 = (self._t_fine / 2) - 64000
        var2 = ((var1 / 4) * (var1 / 4)) / 2048
        var2 = (var2 * self._pressure_calibration[5]) / 4
        var2 = var2 + (var1 * self._pressure_calibration[4] * 2)
        var2 = (var2 / 4) + (self._pressure_calibration[3] * 65536)
        var1 = (
            (((var1 / 4) * (var1 / 4)) / 8192)
            * (self._pressure_calibration[2] * 32)
            / 8
        ) + ((self._pressure_calibration[1] * var1) / 2)
        var1 = var1 / 262144
        var1 = ((32768 + var1) * self._pressure_calibration[0]) / 32768
        calc_pres = 1048576 - self._adc_pres
        calc_pres = (calc_pres - (var2 / 4096)) * 3125
        calc_pres = (calc_pres / var1) * 2
        var1 = (
            self._pressure_calibration[8] * (((calc_pres / 8) * (calc_pres / 8)) / 8192)
        ) / 4096
        var2 = ((calc_pres / 4) * self._pressure_calibration[7]) / 8192
        var3 = (((calc_pres / 256) ** 3) * self._pressure_calibration[9]) / 131072
        calc_pres += (var1 + var2 + var3 + (self._pressure_calibration[6] * 128)) / 16
        return calc_pres / 100

    @property
    def humidity(self) -> float:
        """The relative humidity in RH %"""
        temp_scaled = ((self._t_fine * 5) + 128) / 256
        var1 = (self._adc_hum - (self._humidity_calibration[0] * 16)) - (
            (temp_scaled * self._humidity_calibration[2]) / 200
        )
        var2 = (
            self._humidity_calibration[1]
            * (
                ((temp_scaled * self._humidity_calibration[3]) / 100)
                + (
                    (
                        (
                            temp_scaled
                            * ((temp_scaled * self._humidity_calibration[4]) / 100)
                        )
                        / 64
                    )
                    / 100
                )
                + 16384
            )
        ) / 1024
        var3 = var1 * var2
        var4 = self._humidity_calibration[5] * 128
        var4 = (var4 + ((temp_scaled * self._humidity_calibration[6]) / 100)) / 16
        var5 = ((var3 / 16384) * (var3 / 16384)) / 1024
        var6 = (var4 * var5) / 2
        calc_hum = (((var3 + var6) / 1024) * 1000) / 4096
        calc_hum /= 1000  # get back to RH

        if calc_hum > 100:
            calc_hum = 100
        if calc_hum < 0:
            calc_hum = 0
        return calc_hum

    @property
    def altitude(self) -> float:
        """The altitude based on current ``pressure`` vs the sea level pressure
        (``sea_level_pressure``) - which you must enter ahead of time)"""
        pressure = self.pressure  # in Si units for hPascal
        return 44330.77 * (
            1.0 - math.pow(pressure / self.sea_level_pressure, 0.1902632)
        )

    @property
    def gas(self) -> float:
        """The gas resistance in ohms"""
        var1 = (
            (1340 + (5 * self._sw_err)) * (bme680.LOOKUP_TABLE_1[self._gas_range])
        ) / 65536
        var2 = ((self._adc_gas * 32768) - 16777216) + var1
        var3 = (bme680.LOOKUP_TABLE_2[self._gas_range] * var1) / 512
        calc_gas_res = (var3 + (var2 / 2)) / var2
        return int(calc_gas_res)

    def take_reading(self):
        """Perform a single-shot reading from the sensor and fill internal data structure for
        calculations"""

        # set filter
        self._write(bme680.REG_CONFIG, [self._filter_size << 2])
        # turn on temp oversample & pressure oversample
        self._write(
            bme680.CTRL_MEAS,
            [(self._temp_oversample << 5) | (self._pressure_oversample << 2)],
        )
        # turn on humidity oversample
        self._write(bme680.REG_CTRL_HUM, [self._humidity_oversample])
        # gas measurements enabled
        self._write(bme680.REG_CTRL_GAS, [bme680.RUNGAS])

        ctrl = self._read_byte(bme680.CTRL_MEAS)
        ctrl = (ctrl & 0xFC) | 0x01  # enable single shot!
        self._write(bme680.CTRL_MEAS, [ctrl])
        new_data = False
        while not new_data:
            data = self._read(bme680.REG_MEAS_STATUS, 15)
            new_data = data[0] & 0x80 != 0
            time.sleep(0.01)

        self._adc_pres = float(bme680._bigEndian(data[2:5])) / 16
        self._adc_temp = float(bme680._bigEndian(data[5:8])) / 16
        self._adc_hum = struct.unpack(">H", bytes(data[8:10]))[0]
        self._adc_gas = int(struct.unpack(">H", bytes(data[13:15]))[0] / 64)
        self._gas_range = data[14] & 0x0F

        var1 = (self._adc_temp / 8) - (self._temp_calibration[0] * 2)
        var2 = (var1 * self._temp_calibration[1]) / 2048
        var3 = ((var1 / 2) * (var1 / 2)) / 4096
        var3 = (var3 * self._temp_calibration[2] * 16) / 16384

        self._t_fine = int(var2 + var3)

    def _read_calibration(self):
        """Read & save the calibration coefficients"""
        coeff = self._read(bme680.COEFF_ADDR1, 25)
        coeff += self._read(bme680.COEFF_ADDR2, 16)

        coeff = list(struct.unpack("<hbBHhbBhhbbHhhBBBHbbbBbHhbb", bytes(coeff[1:39])))
        # print("\n\n",coeff)
        coeff = [float(i) for i in coeff]
        self._temp_calibration = [coeff[x] for x in [23, 0, 1]]
        self._pressure_calibration = [
            coeff[x] for x in [3, 4, 5, 7, 8, 10, 9, 12, 13, 14]
        ]
        self._humidity_calibration = [coeff[x] for x in [17, 16, 18, 19, 20, 21, 22]]
        self._gas_calibration = [coeff[x] for x in [25, 24, 26]]

        # flip around H1 & H2
        self._humidity_calibration[1] *= 16
        self._humidity_calibration[1] += self._humidity_calibration[0] % 16
        self._humidity_calibration[0] /= 16

        self._heat_range = (self._read_byte(0x02) & 0x30) / 16
        self._heat_val = self._read_byte(0x00)
        self._sw_err = (self._read_byte(0x04) & 0xF0) / 16

    def _read_byte(self, register):
        raise NotImplementedError()

    def _read(self, register, length):
        raise NotImplementedError()

    def _write(self, register, values):
        raise NotImplementedError()


class BME680_I2C(bme680):
    """
    Driver for I2C connected BME680.
    :param i2c: I2C device (default '/dev/i2c-1')
    :param int address: I2C device address (default 0x76)
    """

    def __init__(self, **kwargs):
        """Initialize the I2C device at the 'address' given"""
        self._i2c = kwargs.get("i2c", 1)
        self._address = kwargs.get("address", 0x76)
        super(BME680_I2C, self).__init__()

    def _read_byte(self, register):
        """Read a byte register value and return it"""
        r = self._read(register, 1)
        return r[0]

    def _read(self, register, length):
        """Returns an array of 'length' bytes from the 'register'"""
        i2c = smbus2.SMBus(self._i2c)
        time.sleep(0.1)
        msgs = [
            smbus2.i2c_msg.write(
                self._address,
                [
                    register,
                ],
            ),
            smbus2.i2c_msg.read(self._address, length),
        ]
        i2c.i2c_rdwr(*msgs)
        return list(msgs[1])

    def _write(self, register, values):
        """Writes an array of 'length' bytes to the 'register'"""
        i2c = smbus2.SMBus(self._i2c)
        time.sleep(0.1)

        msg = smbus2.i2c_msg.write(
            self._address,
            [
                register,
            ]
            + values,
        )
        return i2c.i2c_rdwr(msg)
