#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  radioSerial.py
#  
#  Copyright 2024  <rpi31@rpi31>
#  
#  espejo por uart mensajes entra BT y gpio
#  
#  
import serial
from time import time, sleep
import threading

def serLocalGPIO():
    pGPIO='/dev/serial0'
    #pGPIO='/dev/ttyS0'
    rGPIO=38400
    serGPIO=serial.Serial(
        port=pGPIO,
        baudrate=rGPIO,
        #parity=serial.PARITY_NONE,
        #stopbits=serial.STOPBITS_ONE,
        #bytesize=serial.EIGHTBITS,
        timeout=1
    )
    return serGPIO

def serLocalBT():
    pBT='/dev/ttyAMA0' 
    #pBT='/dev/rfcomm0'
    rBT=9600
    serBT=serial.Serial(
        port=pBT,
        baudrate=rBT,
        #parity=serial.PARITY_NONE,
        #stopbits=serial.STOPBITS_ONE,
        #bytesize=serial.EIGHTBITS,
        timeout=1
    )
    return serBT

def main():
    print('Radio main ha empezado')
    
    empThread=threading.Thread(target=espejo, args=())
    empThread.start()
    
    print('Radio main terminado')
    
    return

def espejo(timeLimit=0):
    #infinite operation as default
    
    print('Espejo empezando')
    
    serBT=serLocalBT()
    serGPIO=serLocalGPIO()
    
    timetick=time()
    continua=True
    while continua:
        if serGPIO.in_waiting>0:
            serBT.write(serGPIO.readline)
        if serBT.in_waiting>0:
            serGPIO.write(serBT.readline)
        sleep(.001)
        if timeLimit>0:
            continua=((time() - timetick)<timeLimit)
    
    print('Espejo ha terminado')
    
    return
    


if __name__ == '__main__':
    main()
    
