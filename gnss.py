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
import struct


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
    


def sendWT(wtL,wtR,timestamp):
    #compose message for sending integer Left and Right Wheel Ticks
    # where 1 WT  = 5 cm of linear travel
    #return nothing
    #credit: https://github.com/ClemensElflein/xbot_driver_gps/blob/main/src/ublox_gps_interface.cpp#L128

    #interpret signed inputs
    wtL=int(wtL)
    wtR=int(wtR)
    #scale
    bit22L=abs(wtL)
    bit22R=abs(wtR)
    #direction
    fwdL=(wtL>0)
    fwdR=(wtR<0)
    endL=0 if fwdL else 1
    endR=0 if fwdR else 1

    
    #Compose and send message
    frame = bytearray(8 + 2 * 4 + 8)
    
    # Set the message class and ID
    frame[2] = 0x10
    frame[3] = 0x02

    #Begin payload construction
    payload = struct.unpack_from('IIII', frame, 6)
    payload[0] = timestamp
    # flags etc, it's all 0
    payload[1] = 0

    data_left = bit22L & 0x7FFFFF
    if fwdL:
        data_left |= 1 << 23
    data_left |= 8 << 24
    payload[2] = data_left

    data_right = bti22R & 0x7FFFFF
    if fwdR:
        data_right |= 1 << 23
    data_right |= 9 << 24
    payload[3] = data_right

    #Send payload
    struct.pack_into('IIII', frame, 6, *payload)
    send_packet(frame, len(frame))
    
    return


if __name__ == '__main__':
    print(iniciar())
    print(getPosition())
