#!/usr/bin/env python3
#-----------------------------------------------------------------------------
# geo_coords_ex1.py
#
# Simple Example for SparkFun ublox GPS products 
#------------------------------------------------------------------------
#
# Written by  SparkFun Electronics, July 2020
# 
# Do you like this library? Help support SparkFun. Buy a board!
# https://sparkfun.com
#==================================================================================
# GNU GPL License 3.0
# Copyright (c) 2020 SparkFun Electronics
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#==================================================================================
# Example 1
# This example sets up the serial port and then passes it to the UbloxGPs
# library. From here we call geo_coords() and to get longitude and latitude. I've
# also included heading of motion here as well. 

import serial

from ublox_gps import UbloxGps
from datetime import timedelta, datetime

port = serial.Serial('/dev/serial0', baudrate=38400, timeout=10)
gps = UbloxGps(port)

def run():

    try:
        print("Listening for UBX Messages")
        p = 0
        s = 0
        f = 0
        t0 = datetime.now()
        td = timedelta(minutes=0, seconds=25)
        t1 = t0 + td
        print(f'Running for {td.seconds} seconds')
        while datetime.now() < t1:
            try:
                geo = gps.geo_coords()
                if not geo is None:
                    print(p, "Longitude: ", geo.lon) 
                    print(p, "Latitude: ", geo.lat)
                    print(p, "Heading of Motion: ", geo.headMot)
                    s += 1
                else:
                    print(p, geo)
                    f += 1
                p += 1
            except (ValueError, IOError) as err:
                print(err)
        print(f'Success in {s} of possible {td.seconds} run ({s/td.seconds:.0%})')
        print(f'Failed messages: {f} ({f/p:.0%} of all messages)')

    finally:
        port.close()


if __name__ == '__main__':
    run()
