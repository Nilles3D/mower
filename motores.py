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
    print('motores  Habilitando motores')
    motors.enable()
    if motors.getFaults():
        if motors.motor1.getFault():
            motNum=1
            motN2=' y 2' if motors.motor2.getFault() else ''
        else:
            motNum=2
            motN2=''
        print(f'motores  !!!Error detectado en motor {motNum}{motN2}. Parando motores')
        motPara()
    return motors.getFaults()
    
def motPara():
    motors.setSpeeds(0,0)
    motors.disable()
    print('motores  Motores parados')
    return 0

def motCtrl(vel1,vel2,prueba=False):
    if prueba:
        print(f'motores dado {vel1}, {vel2}')
    vel1=max(-480,min(int(vel1),480))
    vel2=max(-480,min(int(vel2),480))
    if prueba:
        print(f'motores usar {vel1}, {vel2}')
    motors.setSpeeds(vel1,vel2)
    
    return vel1, vel2


if __name__ == '__main__':
    print('__ Motores ha empezado')
    print(motInic())
    # ~ print(motCtrl(100,100,True))
    for v1 in [0,50,150,300,480]:
        print(motCtrl(v1,0,True))
        sleep(1)
    print(motCtrl(0,0))
    for v2 in [0,-50,-150,-300,-480]:
        print(motCtrl(0,v2,True))
        sleep(1)
    print(motors.getFaults())
    sleep(5)
    motPara()
    ('__ Motores ha parado')
