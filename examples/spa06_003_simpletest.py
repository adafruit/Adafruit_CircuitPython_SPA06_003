# SPDX-FileCopyrightText: Copyright (c) 2025 Tim Cocks for Adafruit Industries
#
# SPDX-License-Identifier: MIT

import time

import board
from adafruit_bus_device.i2c_device import I2CDevice

from adafruit_spa06_003 import SPA06_003, SPA06_003_DEFAULT_ADDR

i2c = board.I2C()
i2c_device = I2CDevice(i2c, SPA06_003_DEFAULT_ADDR)
spa = SPA06_003(i2c_device)


while True:
    if spa.temperature_data_ready and spa.pressure_data_ready:
        print(f"Temperature: {spa.temperature} °C", end="   ")
        print(f"Pressure: {spa.pressure}  hPa")

    time.sleep(1.0)
