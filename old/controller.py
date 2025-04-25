#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  controller.py
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

import evdev

def main(args):
    print('controller   main start')
    
    # ~ for device in evdev.list_devices():
        # ~ print(evdev.InputDevice(device))
    
    ctrler=evdev.InputDevice('/dev/input/event4')
    
    print(ctrler)
    
    for event in ctrler.read_loop():
        print(evdev.categorize(event))
        
    return 0

if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv))
