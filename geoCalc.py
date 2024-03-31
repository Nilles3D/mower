#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  geoCalc.py
#  
#  Copyright 2024  <rpi31@rpi31>
#  
#  calculados por puntos GNSS
#  

import math

altitud=6371*1000+414.561 #m

def numRumbo(p0, p1):
    #given current coord (lat, long) and desired coord
    #returns straight-line heading from p0 to p1
    #credit: mapscaping.com
    
    if len(p0)<=0 or len(p1)<=0:
        return 0
    
    #math
    lat1=math.radians(p0[0])
    long1=math.radians(p0[1])
    lat2=math.radians(p1[0])
    long2=math.radians(p1[1])
    bearing=math.atan2(
        math.sin(long2-long1)*math.cos(lat2),
        math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(long2-long1)
    )
    bearing=math.degrees(bearing)
    bearing=(bearing+360)%360
    
    return bearing

def numGirar(headCurr, headDesired):
    #given current heading and desired heading
    #returns required change in heading to reach desired heading
    
    headTurn=0
    
    #shortest giration
    clockTurn=-(headDesired-headCurr)
    if abs(clockTurn)>180:
        clockTurn=-math.copysign(1,clockTurn)*(360-abs(clockTurn))
    
    headTurn=clockTurn
    #prueba
    # ~ print('When heading '+str(round(headDesired,0))+' deg.')
    # ~ print(' and needing '+str(round(headCurr,0)) +' deg.,')
    # ~ print(' turn '+str(round(headTurn,0))+' deg.')
    
    return headTurn

def numLejo(p0,p1):
    #given two coordinates
    #returns elev.agnostic distance between them
    #credit: omnicalc.com
    
    if len(p0)<=0 or len(p1)<=0:
        return 0
    
    #math
    lat1=math.radians(p0[0])
    long1=math.radians(p0[1])
    lat2=math.radians(p1[0])
    long2=math.radians(p1[1])
    
    a=math.pow(math.sin((lat2-lat1)/2),2)
    b=math.cos(lat1)*math.cos(lat2)*math.pow(math.sin((long2-long1)/2),2)
    distancia=2*altitud*math.asin(math.sqrt(a+b))
    
    return distancia

if __name__ == '__main__':
    print('__PROBANDO geoCalc__')
    
    #prueba numGirar
    #lat=y, long=x
    # ~ cube9=([1,0],[1,1],[0,1],[-1,1],
        # ~ [-1,0],[-1,-1],[0,-1],[1,-1])
    # ~ cube0=[0,0]
    # ~ headSay=[0,45,90,135,180,225,270,315,360]
    # ~ for toPoint in cube9:
        # ~ for headNow in headSay:
            # ~ headActual=numRumbo(cube0,toPoint)
            # ~ headThen=numGirar(headActual,headNow)
    
    #prueba numLejo
    # ~ print(round(numLejo([43,-95],[48,-88]),0))
    # ~ print(numLejo([33,-115],[-18,48]))
    
    print('__prueba completa, geoCalc__')
