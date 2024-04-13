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
import time


#Serial inic
serGPIO=serLocalGPIO()
gps=UbloxGps(serGPIO)

acabar=0.5 #sec

def despertar():
    #Get first GPS response
    gnssUp=False
    timeGnssUp=time.time
    while (not gnssUp) and (time.time-timeGnssUp<=10*acabar):
        rfAnt=gps.rf_ant_status()
        modState=gps.module_wake_state()
        gnssUp=((rfAnt.pAcc>0.1) or (modState.prRes>0.1))
        time.wait(1)
    return gnssUp

def gnssIniciar():
    
    st=despertar()
    
    #basics
    gps_time=gps.date_time()
    gps_time_str='UTC Time {}:{}:{}'.format(gps_time.hour,gps_time.min,gps_time.sec)

    #CFG spec
    #sin desarollo
    
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

def calculate_checksum(ubxcmd):
    #credit: https://github.com/ClemensElflein/xbot_driver_gps/blob/main/src/ublox_gps_interface.cpp#L128
    #credit: https://github.com/cturvey/RandomNinjaChef/blob/main/uBloxRLMwheelticks.c
    ck_a=0
    ck_b=0
    
    for i in ubxcmd:
        ck_a+=i
        ck_b+=ck_a
        
    return ck_a, ck_b

def send_packet(frame, size):
    #credit: https://github.com/ClemensElflein/xbot_driver_gps/blob/main/src/ublox_gps_interface.cpp#L128
    #setup
    frame[0] = 0xb5
    frame[1] = 0x62
    length_ptr = frame[4:6]
    length_ptr[0] = size - 8
    #checksum
    ck_a, ck_b = calculate_checksum(frame[2:size-2])
    frame[size-2] = ck_a
    frame[size-1] = ck_b
    #send straight to serial object
    serGPIO.write(bytes(frame))
    return

def send_UBX(buffer):
    #credit: https://github.com/cturvey/RandomNinjaChef/blob/main/uBloxChecksum.c
    buffLen=size(buffer)
    ckA, ckB=calculate_checksum(buffer)
    buffer[buffLen-2]=ckA
    buffer[buffLen-1]=ckB
    serGPIO.write(bytes(buffer))
    return

def sendWT_xbot(wtL, wtR, ttag):
    #compose message for sending integer Left and Right Wheel Ticks
    # where 1 WT <= 5 cm of linear travel
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
    frame[2] = 0x10 # ESF-MEAS
    frame[3] = 0x02

    #Begin payload construction
    payload = struct.unpack_from('IIII', frame, 6)
    payload[0] = ttag #timestamp
    payload[1] = 0 #flags
    
    left_rear = bit22L & 0x7FFFFF
    left_rear |= 1 << 23 if fwdL
    left_rear |= 8 << 24 # Sensor 8
    payload[2] = left_rear

    right_rear = bit22R & 0x7FFFFF
    right_rear |= 1 << 23 if fwdR
    right_rear |= 9 << 24 # Sensor 9
    payload[3] = right_rear

    #Send payload
    struct.pack_into('IIII', frame, 6, *payload)
    send_packet(frame, len(frame))
    
    return

def sendWT_ninja(left_rear, right_rear, ttag):
    #compose message for sending integer Left and Right Wheel Ticks
    # where 1 WT <= 5 cm of linear travel
    #return nothing
    #credit: https://github.com/ClemensElflein/xbot_driver_gps/blob/main/src/ublox_gps_interface.cpp#L128
    
    length = 16
    ubxcmd = bytearray(8 + 16)
    u = struct.unpack_from('4I', ubxcmd, 6)  # payload

    ubxcmd[0] = 0xB5
    ubxcmd[1] = 0x62

    ubxcmd[2] = 0x10  # ESF-MEAS
    ubxcmd[3] = 0x02

    ubxcmd[4] = length % 256
    ubxcmd[5] = length // 256

    if left_rear & 0x80000000:
        left_rear = -left_rear | 0x800000  # sign to bit 23, positive tick count 0..22
    if right_rear & 0x80000000:
        right_rear = -right_rear | 0x800000

    u[0] = ttag  # Timestamp in your MCU time-line, receiver will add it's own upon reception
    u[1] = (2 << 11)  # Flags/Id, Two Sensors
    u[2] = (left_rear & 0x00FFFFFF) | (8 << 24)  # Sensor 8 : Left-Rear Wheel Tick
    u[3] = (right_rear & 0x00FFFFFF) | (9 << 24)  # Sensor 9 : Right-Rear Wheel Tick

    # Sums and sends...
    send_UBX(ubxcmd)
    return

if __name__ == '__main__':
    print(iniciar())
    print(getPosition())
