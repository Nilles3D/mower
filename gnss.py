#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  gnss.py
#  
#  Copyright 2024  <rpi31@rpi31>
#  
#  todo gnss
#  
#  
from ublox_gps import UbloxGps
from radioSerial import serLocalGPIO


#Serial inic
serGPIO=serLocalGPIO()
gps=UbloxGps(serGPIO)

acabar=0.5 #sec

def despertar():
    #Get first GPS lock
    gnssLock=False
    timeGnssLock=time.time
    while (not gnssLock) and (time.time-timeGnssLock<=10*acabar):
        #prompt startups
        rfAnt=gps.rf_ant_status()
        modState=gps.module_wake_state()
        #gnssLock=((rfAnt.pAcc>0.1) or (modState.prRes>0.1))
        time.wait(0.1)
    return gnssLock

def gnssIniciar():
    
    st=despertar()
    
    #basics
    gps_time=gps.date_time()
    gps_time_str='UTC Time {}:{}:{}'.format(gps_time.hour,gps_time.min,gps_time.sec)
    
    return gps_time_str

def gnssParar():
    
    serGPIO.close()
    
    return

def getHeading():
    
    gObj=gps.geo_coords()
    myHeading=gObj.headMot#.heading?
    
    return myHeading

def getPosition():
    
    gObj=gps.geo_coords()
    myPosition=[gObj.lat,gObj.lon]
    
    return myPosition
    
def sendWT(wtL,wtR):
    #compose message for sending integer Left and Right Wheel Ticks
    # where 1 WT  = 5 cm of linear travel
    #return nothing
    
    wtL=int(wtL)
    wtR=int(wtR)
    
    endL=0 if wtL>0 else 1
    endR=0 if wtR>0 else 1
    
    bit22L=wtL
    bit22R=wtR
    
    #compose and send message
    
    
    return

if __name__ == '__main__':
    print(iniciar())
    print(getPosition())
