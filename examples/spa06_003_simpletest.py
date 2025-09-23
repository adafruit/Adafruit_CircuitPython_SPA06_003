# SPDX-FileCopyrightText: Copyright (c) 2025 Tim Cocks for Adafruit Industries
#
# SPDX-License-Identifier: MIT

import time

import board

from adafruit_spa06_003 import SPA06_003_I2C

i2c = board.I2C()
spa = SPA06_003_I2C(i2c)


while True:
    if spa.temperature_data_ready and spa.pressure_data_ready:
        print(f"Temperature: {spa.temperature} °C", end="   ")
        print(f"Pressure: {spa.pressure}  hPa")

    time.sleep(0.01)
