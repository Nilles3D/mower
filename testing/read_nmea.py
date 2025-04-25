#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  read_nmea.py
#  
#  Copyright 2024  <rpi31@rpi31>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#  


def main(args):
    import serial
    from pynmeagps import NMEAReader
    print('ini')
    with serial.Serial(port='/dev/serial0', baudrate=38400, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=3) as stream:
        x=0
        while x<10:
            nmr=NMEAReader(stream)
            raw_data, parsed_data = nmr.read()
            if parsed_data is not None:
                print(parsed_data)
            x+=1
    print('fin')
    return 0

if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv))
