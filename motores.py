#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  motores.py
#  
#  Copyright 2024  <rpi31@rpi31>
#  
#  motor command parsing and execution
#  

from dual_tb9051ftg_rpi import motors
from time import sleep

def motInic():
    motors.enable()
    if motors.getFaults():
        print('!!!Error detectado. Parando motores')
        motPara
    return motors.getFaults()
    
def motPara():
    motors.setSpeeds(0,0)
    motors.disable()
    print('Motores parados')
    return 0

def motCtrl(vel1,vel2):
    vel1=max(-480,min(int(vel1),480))
    vel2=max(-480,min(int(vel2),480))
    motors.setSpeeds(vel1,vel2)
    
    return vel1, vel2


if __name__ == '__main__':
    print('Motores ha empezado')
    motInic
    motCtrl(200,-200)
    sleep(2)
    motPara
    ('Motores ha parado')
