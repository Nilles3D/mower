#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  probar.py
#  
#  Copyright 2024  <rpi32@rpi32>
#  
#  

import gpiozero as gp
pinB=gp.LED(19)

from time import sleep

def main(args):
    return 0

if __name__ == '__main__':
    # ~ pinB.blink(on_time=1,off_time=1,n=2)
    # ~ pinB.off()
    # ~ sleep(6)
    # ~ pinB.close()
    
    ms="b'\x02'"
    mc=eval(ms)
    mi='abc'
    mc=mi.encode()
    print(mc)
    print(mc.decode())
    
