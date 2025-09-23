# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2025 Tim Cocks for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_spa06_003`
================================================================================

CircuitPython driver for SPA06-003 temperature and pressure sensor breakout.


* Author(s): Tim Cocks

Implementation Notes
--------------------

**Hardware:**

.. todo:: Add links to any specific hardware product page(s), or category page(s).
  Use unordered list & hyperlink rST inline format: "* `Link Text <url>`_"

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

.. todo:: Uncomment or remove the Bus Device and/or the Register library dependencies
  based on the library's use of either.

# * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
# * Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

# imports

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_SPA06_003.git"

import time
from micropython import const
from adafruit_register.i2c_bits import ROBits, RWBits
from adafruit_register.i2c_bit import ROBit, RWBit
from adafruit_register.i2c_struct import ROUnaryStruct
from adafruit_bus_device.i2c_device import I2CDevice

SPA06_003_DEFAULT_ADDR = const(0x77)
SPA06_003_ALTERNATE_ADDR = const(0x76)
SPA06_003_DEFAULT_SPIFREQ = const(1000000)  # 1 MHz

# Pressure data registers
SPA06_003_REG_PSR_B2 = const(0x00)  # Pressure data byte 2 (MSB)
SPA06_003_REG_PSR_B1 = const(0x01)  # Pressure data byte 1
SPA06_003_REG_PSR_B0 = const(0x02)  # Pressure data byte 0 (LSB)

# Temperature data registers
SPA06_003_REG_TMP_B2 = const(0x03)  # Temperature data byte 2 (MSB)
SPA06_003_REG_TMP_B1 = const(0x04)  # Temperature data byte 1
SPA06_003_REG_TMP_B0 = const(0x05)  # Temperature data byte 0 (LSB)

# Configuration registers
SPA06_003_REG_PRS_CFG = const(0x06)  # Pressure configuration register
SPA06_003_REG_TMP_CFG = const(0x07)  # Temperature configuration register
SPA06_003_REG_MEAS_CFG = const(0x08)  # Measurement configuration register
SPA06_003_REG_CFG_REG = const(0x09)  # General configuration register

# Status registers
SPA06_003_REG_INT_STS = const(0x0A)  # Interrupt status register
SPA06_003_REG_FIFO_STS = const(0x0B)  # FIFO status register

# Control registers
SPA06_003_REG_RESET = const(0x0C)  # Reset register
SPA06_003_REG_ID = const(0x0D)  # Chip ID register

# Calibration coefficient registers
SPA06_003_REG_COEF_C0 = const(0x10)  # Calibration coefficients C0 start address
SPA06_003_REG_COEF_C1 = const(0x11)  # Calibration coefficients C1 start address
SPA06_003_REG_COEF_C00 = const(0x13)  # Calibration coefficients C00 start address
SPA06_003_REG_COEF_C10 = const(0x15)  # Calibration coefficients C10 start address
SPA06_003_REG_COEF_C01 = const(0x18)  # Calibration coefficients C01 start address
SPA06_003_REG_COEF_C11 = const(0x1A)  # Calibration coefficients C11 start address
SPA06_003_REG_COEF_C20 = const(0x1C)  # Calibration coefficients C20 start address
SPA06_003_REG_COEF_C21 = const(0x1E)  # Calibration coefficients C21 start address
SPA06_003_REG_COEF_C30 = const(0x20)  # Calibration coefficients C30 start address
SPA06_003_REG_COEF_C31 = const(0x22)  # Calibration coefficients C31 start address
SPA06_003_REG_COEF_C40 = const(0x23)  # Calibration coefficients C40 start address

# Interrupt status flag definitions
SPA06_003_INT_FIFO_FULL = const(0x04)  # FIFO full flag
SPA06_003_INT_TMP_RDY = const(0x02)  # Temperature measurement ready flag
SPA06_003_INT_PRS_RDY = const(0x01)  # Pressure measurement ready flag

SPA06_003_CMD_RESET = const(0x09)

# Measurement rate options
SPA06_003_RATE_1 = const(0x00)  # 1 measurements per second
SPA06_003_RATE_2 = const(0x01)  # 2 measurements per second
SPA06_003_RATE_4 = const(0x02)  # 4 measurements per second
SPA06_003_RATE_8 = const(0x03)  # 8 measurements per second
SPA06_003_RATE_16 = const(0x04)  # 16 measurements per second
SPA06_003_RATE_32 = const(0x05)  # 32 measurements per second
SPA06_003_RATE_64 = const(0x06)  # 64 measurements per second
SPA06_003_RATE_128 = const(0x07)  # 128 measurements per second
SPA06_003_RATE_25_16 = const(0x08)  # 25/16 samples per second
SPA06_003_RATE_25_8 = const(0x09)  # 25/8 samples per second
SPA06_003_RATE_25_4 = const(0x0A)  # 25/4 samples per second
SPA06_003_RATE_25_2 = const(0x0B)  # 25/2 samples per second
SPA06_003_RATE_25 = const(0x0C)  # 25 samples per second
SPA06_003_RATE_50 = const(0x0D)  # 50 samples per second
SPA06_003_RATE_100 = const(0x0E)  # 100 samples per second
SPA06_003_RATE_200 = const(0x0F)  # 200 samples per second

# Oversampling rate options (shared by pressure and temperature)
SPA06_003_OVERSAMPLE_1 = const(0x00)  # Single
SPA06_003_OVERSAMPLE_2 = const(0x01)  # 2 times
SPA06_003_OVERSAMPLE_4 = const(0x02)  # 4 times
SPA06_003_OVERSAMPLE_8 = const(0x03)  # 8 times
SPA06_003_OVERSAMPLE_16 = const(0x04)  # 16 times
SPA06_003_OVERSAMPLE_32 = const(0x05)  # 32 times
SPA06_003_OVERSAMPLE_64 = const(0x06)  # 64 times
SPA06_003_OVERSAMPLE_128 = const(0x07)  # 128 times

# Measurement mode options
SPA06_003_MEAS_IDLE = const(0x00)  # Idle / Stop background measurement
SPA06_003_MEAS_PRESSURE = const(0x01)  # Pressure measurement (Command Mode)
SPA06_003_MEAS_TEMPERATURE = const(0x02)  # Temperature measurement (Command Mode)
SPA06_003_MEAS_CONTINUOUS_PRESSURE = const(0x05)  # Continuous pressure measurement (Background Mode)
SPA06_003_MEAS_CONTINUOUS_TEMPERATURE = const(0x06)  # Continuous temperature measurement (Background Mode)
SPA06_003_MEAS_CONTINUOUS_BOTH = const(0x07)  # Continuous pressure and temperature measurement (Background Mode)

# Interrupt polarity options
SPA06_003_INT_ACTIVE_LOW = const(0x00)  # Interrupt active low
SPA06_003_INT_ACTIVE_HIGH = const(0x01)  # Interrupt active high

# Scaling factor lookup dictionary
SPA06_003_SCALING_FACTORS_LUT = {
    SPA06_003_OVERSAMPLE_1: 524288,  # Single
    SPA06_003_OVERSAMPLE_2: 1572864,  # 2x
    SPA06_003_OVERSAMPLE_4: 3670016,  # 4x
    SPA06_003_OVERSAMPLE_8: 7864320,  # 8x
    SPA06_003_OVERSAMPLE_16: 253952,  # 16x
    SPA06_003_OVERSAMPLE_32: 516096,  # 32x
    SPA06_003_OVERSAMPLE_64: 1040384,  # 64x
    SPA06_003_OVERSAMPLE_128: 2088960,  # 128x
}


class SPA06_003_I2C:
    chip_id = ROBits(8, SPA06_003_REG_ID, 0)

    soft_reset_cmd = RWBits(4, SPA06_003_REG_RESET, 0)

    coeff_ready = ROBit(SPA06_003_REG_MEAS_CFG, 7)
    sensor_ready = ROBit(SPA06_003_REG_MEAS_CFG, 6)

    _coeff_c0 = ROBits(12, SPA06_003_REG_COEF_C0, 4, register_width=2, lsb_first=False, signed=True)
    _coeff_c1 = ROBits(12, SPA06_003_REG_COEF_C1, 0, register_width=2, lsb_first=False, signed=True)
    _coeff_c00 = ROBits(20, SPA06_003_REG_COEF_C00, 4, register_width=3, lsb_first=False, signed=True)
    _coeff_c10 = ROBits(20, SPA06_003_REG_COEF_C10, 0, register_width=3, lsb_first=False, signed=True)
    _coeff_c01 = ROBits(16, SPA06_003_REG_COEF_C01, 0, register_width=2, lsb_first=False, signed=True)
    _coeff_c11 = ROBits(16, SPA06_003_REG_COEF_C11, 0, register_width=2, lsb_first=False, signed=True)
    _coeff_c20 = ROBits(16, SPA06_003_REG_COEF_C20, 0, register_width=2, lsb_first=False, signed=True)
    _coeff_c21 = ROBits(16, SPA06_003_REG_COEF_C21, 0, register_width=2, lsb_first=False, signed=True)
    _coeff_c30 = ROBits(16, SPA06_003_REG_COEF_C30, 0, register_width=2, lsb_first=False, signed=True)
    _coeff_c31 = ROBits(12, SPA06_003_REG_COEF_C31, 4, register_width=2, lsb_first=False, signed=True)
    _coeff_c40 = ROBits(12, SPA06_003_REG_COEF_C40, 0, register_width=2, lsb_first=False, signed=True)

    _pressure_oversampling = RWBits(3, SPA06_003_REG_PRS_CFG, 0)
    pressure_shift_enabled = RWBit(SPA06_003_REG_CFG_REG, 2)

    pressure_measure_rate = RWBits(4, SPA06_003_REG_PRS_CFG, 4)

    _temperature_oversampling = RWBits(3, SPA06_003_REG_TMP_CFG, 0)
    temperature_shift_enabled = RWBit(SPA06_003_REG_CFG_REG, 3)

    temperature_measure_rate = RWBits(4, SPA06_003_REG_TMP_CFG, 4)

    fifo_interrupt = RWBit(SPA06_003_REG_CFG_REG, 6)
    temperature_interrupt = RWBit(SPA06_003_REG_CFG_REG, 5)
    pressure_interrupt = RWBit(SPA06_003_REG_CFG_REG, 4)

    measurement_mode = RWBits(3, SPA06_003_REG_MEAS_CFG, 0)

    temperature_data_ready = ROBit(SPA06_003_REG_MEAS_CFG, 5)
    pressure_data_ready = ROBit(SPA06_003_REG_MEAS_CFG, 4)

    _temperature_bits = ROBits(24, SPA06_003_REG_TMP_B2, 0, register_width=3, lsb_first=False, signed=True)
    _temperature = ROUnaryStruct(SPA06_003_REG_TMP_B2, "<3s")

    def __init__(self, i2c_bus, address=SPA06_003_DEFAULT_ADDR):
        try:
            self.i2c_device = I2CDevice(i2c_bus, address)
        except ValueError:
            raise ValueError(f"No I2C device found at address 0x{address:02X}")

        if self.chip_id != 0x11:
            raise ValueError("SPA06_003_I2C device not found")

        self.reset()

        while not self.sensor_ready or not self.coeff_ready:
            time.sleep(0.01)
        print("Sensor and Coefficients ready.")

        # Coefficient values do not change, so just read them once
        self.coeff_c0 = self._coeff_c0
        self.coeff_c1 = self._coeff_c1
        self.coeff_c00 = self._coeff_c00
        self.coeff_c10 = self._coeff_c10
        self.coeff_c01 = self._coeff_c01
        self.coeff_c11 = self._coeff_c11
        self.coeff_c20 = self._coeff_c20
        self.coeff_c21 = self._coeff_c21
        self.coeff_c30 = self._coeff_c30
        self.coeff_c31 = self._coeff_c31
        self.coeff_c40 = self._coeff_c40

        # Configure for highest precision and sample rate
        # Set pressure to highest oversampling (128x) and highest rate (200 Hz)
        self.pressure_oversampling = SPA06_003_OVERSAMPLE_128
        self.pressure_measure_rate = SPA06_003_RATE_200

        # Set temperature to highest oversampling (128x) and highest rate (200 Hz)
        self.temperature_oversampling = SPA06_003_OVERSAMPLE_128
        self.temperature_measure_rate = SPA06_003_RATE_200

        # Enable interrupts for temperature and pressure ready
        self.fifo_interrupt = False
        self.pressure_interrupt = True
        self.temperature_interrupt = True

        # Set measurement mode to continuous both
        self.measurement_mode = SPA06_003_MEAS_CONTINUOUS_BOTH


    @property
    def temperature(self):
        temp_raw_signed = self._temperature_bits
        kT = SPA06_003_SCALING_FACTORS_LUT.get(self.temperature_oversampling, 524288)
        temp_raw_sc = float(temp_raw_signed) / kT
        temp_comp = float(self.coeff_c0) * 0.5 + float(self.coeff_c1) * temp_raw_sc

        return temp_comp

    @property
    def pressure_oversampling(self):
        return self._pressure_oversampling

    @pressure_oversampling.setter
    def pressure_oversampling(self, new_value):
        if new_value not in SPA06_003_SCALING_FACTORS_LUT.keys():
            raise ValueError("Oversampling must be one of the OVERSAMPLING constants")

        self._pressure_oversampling = new_value
        self.pressure_shift_enabled = new_value > SPA06_003_OVERSAMPLE_8

    @property
    def temperature_oversampling(self):
        return self._temperature_oversampling

    @temperature_oversampling.setter
    def temperature_oversampling(self, new_value):
        if new_value not in SPA06_003_SCALING_FACTORS_LUT.keys():
            raise ValueError("Oversampling must be one of the OVERSAMPLING constants")

        self._temperature_oversampling = new_value
        self.temperature_shift_enabled = new_value > SPA06_003_OVERSAMPLE_8

    def reset(self):
        self.soft_reset_cmd = SPA06_003_CMD_RESET
        time.sleep(0.01)

    def debug_temperature_read(self):
        # Read raw bytes manually
        buffer = bytearray(4)
        buffer[0] = SPA06_003_REG_TMP_B2
        with self.i2c_device as i2c:
            i2c.write_then_readinto(buffer, buffer, out_end=1, in_start=1)

        print(f"Raw bytes: {[hex(b) for b in buffer[1:]]}")

        # Reconstruct the 24-bit value manually (MSB first)
        raw_value = (buffer[1] << 16) | (buffer[2] << 8) | buffer[3]
        print(f"Raw 24-bit value: {raw_value} (0x{raw_value:06x})")

        # Apply sign extension like Arduino does
        if raw_value & 0x800000:
            raw_value |= 0xFF000000
        signed_value = raw_value if raw_value < 0x80000000 else raw_value - 0x100000000
        print(f"Sign-extended value: {signed_value}")
        return signed_value
