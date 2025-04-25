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
    print(f'radioSerial  Empezando serLocalGPIO con {rGPIO} en {pGPIO}')
    serGPIO=serial.Serial(
        port=pGPIO,
        baudrate=rGPIO,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )
    return serGPIO

def serLocalBT():
    pBT='/dev/ttyAMA0' 
    #pBT='/dev/rfcomm0'
    rBT=115200#9600?
    print(f'radioSerial  Empezando serLocalBT con {rBT} en {pBT}')
    serBT=serial.Serial(
        port=pBT,
        baudrate=rBT,
        #parity=serial.PARITY_NONE,
        #stopbits=serial.STOPBITS_ONE,
        #bytesize=serial.EIGHTBITS,
        timeout=1
    )
    return serBT
    
def serLocalUSB():
    pUSB='/dev/ttyACM0'
    rUSB=38400
    print(f'radioSerial  Empezando serLocalUSB con {rUSB} en {pUSB}')
    serUSB=serial.Serial(
        port=pUSB,
        baudrate=rUSB,
        timeout=1
    )
    return serUSB
    

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
    #serGPIO=serLocalGPIO()
    serUSB=serLocalUSB()
    
    timetick=time()
    continua=True
    while continua:
        if serGPIO.in_waiting>0:
            # ~ serBT.write(serGPIO.readline)
        # ~ if serBT.in_waiting>0:
            # ~ serGPIO.write(serBT.readline)
            serUSB.write(serGPIO.readline)
            #serUSB.flushInput()
        if serUSB.in_waiting>0:
            serGPIO.write(serUSB.readline)
            #serGPIO.flushInput()
        sleep(.001)
        if timeLimit>0:
            continua=((time() - timetick)<timeLimit)
    
    print('Espejo ha terminado')
    
    return
    


if __name__ == '__main__':
    main()
    
