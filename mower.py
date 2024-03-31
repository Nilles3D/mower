#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  mower.py
#  
#  Copyright 2024  <rpi31@rpi31>
#  
#  
#  
#  

#custom
import motores
import gnss
import geoCalc as gc
import fileMan
#standard
from simple_pid import PID
import time
import datetime
import gpiozero as gp
import os
from subprocess import call
import statistics
import threading


#globals
arcRuta=os.path.dirname(os.path.abspath(__file__))+'/'#/home/rpi31/Documents/
radiusMatch=11*0.0254 #m
rato=0.1 #sec, 10Hz RTN limit
velStd=0.5 #m/s
velL=0 #/480
velR=0 #/480
#off time for leds
flashShort=0.5 #sec
flashMedium=1.0 #sec
flashLong=3.0 #sec
onTime=0.5 #sec
#threading
motoPara=threading.Event()

#pinouts
#GPIO No., (board pin)
'''
3V3 (1) gnss power
5 (29) motor diagonals
6 (30) motor diagonals
12 (32) motor PWM
13 (33) motor PWM
14 (8) gnss UART
15 (10) gnss UART
22 (15) motor enable
23 (16) motor enable
24 (18) motor direction
25 (22) motor direction
'''
ledPower=gp.LED(21) #rojo
ledProcess=gp.LED(20) #amarillo, recordar o autonomo
ledGNSS=gp.LED(26) #azul
ledMotores=gp.LED(16) #verde
ledBeacon=gp.LED(19) #beacon's relay
btnRecordar=gp.Button(pin=27,pull_up=True)
btnCortar=gp.Button(pin=17,pull_up=True)
potLeft=gp.MCP3002(channel=0)
potRight=gp.MCP3002(channel=1)


def ctrlLuces(io): 
    #given requested bool state of all lights/sounds
    #returns nothing
    
    todoPins=(ledPower,ledProcess,ledGNSS,ledBeacon,ledMotores)
    
    #luces, camera, accion
    #por todos
    for l in todoPins:
        if io:
            l.on()
        else:
            l.off()
        
    return

def cambiaGirar(pidIN,velL,velR):
    #given a PID calculated input based on heading delta & current velocities
    # scales value to motor board's prefered map
    # commands a divergence in motor radial velocity
    #  positive required differential = positive change to left, negated to right
    #returns commanded velocities per motor
    
    delRueda=-480*pidIN #allows for over-commanding
    velL+=delRueda
    velR-=delRueda
    motores.motCtrl(velL,velR)
    
    return velL,velR

def numRollingAvg(myList,myIndex,myValue):
    #given list of numbers, index of list to update, value to update that to
    #returns list with changed number, the next index in sequence, and the average of the changed list
    
    try:
        myList[myIndex]=myValue
        myIndex+=1
        if myIndex>len(myList)-1: myIndex=0
        rolledAvg=statistics.fmean(myList)
    except:
        rolledAvg=None
    
    return myList, myIndex, rolledAvg

def cambiaControl(ratoSus=0):
    #given timeout for when all controls are at zero state
    # passes rheostat inputs to motor control. FORWARD ONLY
    # intended as an independent thread
    #returns nothing
    
    #inic
    timeCambia=time.time
    ratoFin=False
    shutStart=False
    #suave
    histLeft=[0 for i in range(20)]
    histRight=histLeft
    histIndexL=0
    histindexR=0
    
    while (btnRecordar.is_pressed==True) and (not motoPara.is_set()) and (not ratoFin):
        #read and map pot switches to controller values
        adcLeft=(potLeft.value-500)/(1500-500)*480
        adcRight=(potRight.value-500)/(1500-500)*480
        histLeft,histIndexL,avgLeft=numRollingAvg(histLeft,histIndexL,adcLeft)
        histRight,histIndexR,avgRight=numRollingAvg(histRight,histIndexR,adcRight)
        #send inputs to controller
        velL,velR=motores.motCtrl(avgLeft,avgRight)
        
        #throw timeout flag
        if shutStart and ((avgLeft<0.001) and (avgRight<0.001)):
            timeCambia=time.time
            shutStart=False
        else:
            shutStart=True
        ratoFin=((ratoSus>0) and (time.time-timeCambia>ratoSus))
        #don't overload anything
        time.sleep(rato)
    
    #shutdown
    ledMotores.blink(on_time=onTime,off_time=flashShort,n=4)
    ledMotores.off()
    
    return

def enviarTicks():
    #regularly send wheel ticks (WT) over serial to GNSS
    #return nothing. Intended as independent thread
    
    rato1=1/10#s, GNSS receiving limit
    tickMult=(50*(2*3.14159/1)*(1/60)*(34.4/2))/5/480*rato1
        #(max RPM*(2*pi rad / rev)*(min / 60 s)*(dia cm/2))/5cm/max command*time
    
    while not motoPara.is_set():
        time.sleep(rato1)
        
        #normal forward = positive
        wtL=round(velL*tickMult,0)
        wtR=-round(velR*tickMult,0)
        
        gnss.sendWT(wtL,wtR)
    
    return

def mowerInic():
    #startup sequence for any mowing op
    #returns general status
    
    sinErrores=True
    
    #Startup and Signaling
    #start engines
    motores.motInic()
    #warm up GNSS
    gnss.gnssIniciar() #avoid wake wait?
    wtThread=threading.Thread(target=enviarTicks,args=())
    wtThread.start()
    #start led strip
    ctrlLuces(True) #todos
    time.sleep(flashShort)
    ctrlLuces(False)
    ledPower.on()
    
    #Choose the Mode
    #follow toggle orders
    horaVia=time.time
    archivoGuia=arcRuta+'rutaSeguida.txt'
    opcion=False
    
    #start thread of motor control
    motThread=threading.Thread(target=cambiaControl, args=())
    motThread.start()
    
    #wait for gnss lock
    time.sleep(30)
    
    while horaVia-time.time<240 and not opcion:
        #read possible switches
        if btnRecordar.is_pressed: #record, or
            opcion=True
            #indicate lights
            ledProcess.blink(onTime, flashShort)
            #record location of guide
            sinErrores, archivoNuevo=recordar()
            recArchivo=open(archivoGuia,'w')
            recArchivo.write(archivoNuevo)
            recArchivo.close()
            
        elif btnCortar.is_pressed: #cut
            opcion=True
            motoPara.set()
            #indicate lights
            ledProcess.on()
            #get most recent point list
            with open(archivoGuia,'r') as arcAct:
                archivoActual=arcAct.readline()
                #arcAct.close()#quiza?
            datPointList=[]
            #read in the list
            with open(archivoActual,'r') as arcSrc:
                for linea in arcSrc:
                    d=linea[:-1]
                    l=d.split()
                    ll=[float(x) for x in l]
                    datPointList.append(ll)
            #cut
            sinErrores=cortar(datPointList)
        
        else: #wait for human to start
            time.sleep(1)
    
    if not opcion: #manual operation only
        gnss.gnssParar()
        ledProcess.off()
        ledMotores.blink(onTime,flashMedium)
        ledBeacon.blink(onTime,flashShort,n=1)
        
    
    return sinErrores

def mowerPara(errStop):
    
    #cut engines
    motores.motPara()
    motoPara.set()
    #release GNSS
    gnss.gnssParar()
    #stop lights
    ctrlLuces(False)
    if errStop:
        mowerLog.close()
        ledBeacon.blink(on_time=onTime,off_time=flashShort,n=60)
        
    return

def recordar(prlist=[]):
    #record set of coordinates as a list to a local file
    #returns general status and full path of file made
    
    hazRec=True
    
    #prep file for recording
    archivoRuta=arcRuta+'rutas/'
    archivoFecha=str(datetime.date.today())
    #check for any copy names
    archivoNombre=fileMan.prefixMatch(archivoRuta,archivoFecha)
    #set name
    archivoRuta+=archivoNombre+'.txt'
    
    #record points to follow
    with open(archivoRuta,'w') as a:
        ddV=[0.1,0.1]
        #wait for button debounce
        #while btnRecordar.is_pressed==False:
            #time.sleep(rato)
        #for dd in prlist: #testing
        while btnRecordar.is_pressed==True:
            time.sleep(2*rato)
            dd=gnss.getPosition()
            #skip exact same recordings
            if not dd==ddV:
                datu=' '.join(str(ll) for ll in dd)
                a.write(datu+'\n')
            ddV=dd
        
    #complete
    motoPara.set() #backup
    motThread.join()
    ledProcess.on()
    
    
    return hazRec, archivoRuta

def cortar(datPointList):
    #follows given coordinate list
    #returns general status
    
    #Scope init
    #goMow=mowerInic()
    goMow=True
    #ruedas
    ruedaL=0
    ruedaR=0 #neg. forward
    pid=PID(1,0.1,0.05,setpoint=0)
    
    #GNSS locked
    ledGNSS.blink(onTime, flashLong)
    ledBeacon.on()
    
    
    #First spin to first point
    headDel=180
    #get intended positions
    datPointNext=datPointList[0] #lat, long
    while goMow and abs(headDel)>5:
        #get current position
        gpsPointPrev=gnss.getPosition() #brujula degrados: lat, long
        #determine required heading to intended position
        rumPrimera=gc.numRumbo(gpsPointPrev,datPointNext) #deg
        gpsHead=gnss.getHeading() #deg
        headDel=gc.numGirar(gpsHead,rumPrimera) #deg
        #rotate to heading
        pidR=pid(headDel)
        ruedaL,ruedaR=cambiaGirar(pidR,ruedaL,ruedaR)
        
    
    #Sumer velocidad
    #ramp up wheel speed to 75% max or until intended position is reached
    lejoStart=4*radiusMatch
    spdPerc=0
    while goMow and spdPerc<=0.75 and lejoStart>2*radiusMatch:#m
        spdPerc+=0.1
        lejoStart=gc.numLejo(gnss.getPosition,datPointNext)
        ruedaL,ruedaR=cambiaGirar(spdPerc,ruedaL,ruedaR)
        time.sleep(0.1)
    #kick speed up if started too close to first point
    if ruedaL<=0.25*480:
        ruedaL=0.75*480
        ruedaR=-ruedaL
    
    
    #Iniciar cortar
    targetReached=False
    errPerd=0
    velCurr=velStd
    velAvg=[velCurr for i in range(100)]
    velAvgIndex=0
    timeSpdPrev=time.time
    #signal and start
    ledProcess.blink(onTime,flashLong)
    for datPoint in datPointList:
        
        while goMow and not targetReached:
            
            #GNSS positioning
            gpsPointCurr=gnss.getPosition() #lat, long
            distToNextPoint=gc.numLejo(gpsPointCurr,datPoint) #m
            degHeadingIntend=gc.numRumbo(gpsPointCurr,datPoint) #brujula degrees
            
            #Continue if point is in front of mower
            if degHeadingIntend<110:
            
                #GNSS headings
                #get gnss given heading
                degHeadingGNSS=gnss.getHeading() #brujula degrees
                timeSpdCurr=time.time #for later
                #determine actual heading
                degHeadingActual=gc.numRumbo(gpsPointPrev,gpsPointCurr) #deg
                if abs(numGirar(degHeadingActual,degHeadingGNSS))>30:
                    print('!!! GNSS que necesita ajuste, o giramos rapidamente.')
                    print('     Rumbo observado: '+str(round(degHeadingGNSS)))
                    print('     Rumbo calculado: '+str(round(degHeadingActual)))
                #give shortest spin angle to heading
                degChangeReq=gc.numGirar(degHeadingActual,degHeadingIntend) #deg
                            
                #Twist the wheels
                #check centerline speed to standard
                velCurr=gc.numLejo(gpsPointPrev,gpsPointCurr)/(timeSpdCurr-timeSpdPrev)#m/s
                timeSpdPrev=time.time
                velAvg,velAvgIndex,velAvgAvg=numRollingAvg(velAvg,velAvgIndex,velCurr)
                if velAvgAvg<velStd:
                    ruedaL*=1.1
                    ruedaR*=1.1
                    ruedaL,ruedaR=motores.motCtrl(ruedaL,ruedaR)
                elif velAvgAvg>=1.5*velStd:
                    ruedaL*=0.9
                    ruedaR*=0.9
                    ruedaL,ruedaR=motores.motCtrl(ruedaL,ruedaR)
                #input of heading differential (degree value amount needed)
                pidR=pid(degChangeReq)
                ruedaL,ruedaR=cambiaGirar(pidR,ruedaL,ruedaR)
                
                #reset
                errPerd=0
                gpsPointPrev=gpsPointCurr
                targetReached=(distToNextPoint<=radiusMatch)
            
            else: #regard the point as a miss
                errPerd+=1
                targetReached=True
            
            if errPerd>5:
                print('!!! Muchos puntos perdidos en sequencia. Parandome')
                goMow=False
            
            #continua a punto
    #sigue con proximo punto
                
    #Cortar completa
    return goMow



if __name__ == '__main__':
    print('__PROBANDO__ mower')
    
    print(datetime.date.today())
    
    #probando recordar
    # ~ datPointList=([44.4,-88.8],[45.5,-89.9],[46.6,-90.0]) #lat, longs
    # ~ for dd in datPointList:
        # ~ print(dd)
    # ~ tf,archivoActual=recordar(datPointList)
    
    #probando leer
    # ~ archivoActual='/home/rpi31/Documents/rutas/2024-02-20 5.txt'
    # ~ datPointList=[]
    # ~ #read in the list
    # ~ with open(archivoActual,'r') as arcSrc:
        # ~ for linea in arcSrc:
            # ~ d=linea[:-1]
            # ~ l=d.split()
            # ~ ll=[float(x) for x in l]
            # ~ datPointList.append(ll)
    # ~ for dd in datPointList:
        # ~ print(dd)
        # ~ print(dd[0]+1)
        
    #probando numRollingAvg
    # ~ chkList=[i for i in range(5)]
    # ~ k=0
    # ~ for j in chkList:
        # ~ chkList,k,a=numRollingAvg(chkList,k,j+1)
        # ~ print(chkList)
        # ~ print(a)
        
    # ~ print(os.path.dirname(os.path.abspath(__file__)))
    
    #probando luces y pot
    ctrlLuces(True)
    time.sleep(1)
    ctrlLucs(False)1
    cambiaControl(10)
    
    print('__prueba completa__')
else:
    fecha=datetime.datetime.now()
    
    import sys
    stdoutOld=sys.stdout
    fechaLog=fileMan.prefixMatch(arcRuta+'logs/',str(fecha))
    mowerLog=open(fechaLog+'.txt','w')
    sys.stdout=mowerLog
    
    print('__AUTONOMY__ mower')
    print(fecha)
    hora0=time.time
    sinErrores=mowerInic()
    mowerPara(not sinErrores)
    horaTotal=time.time-hora0
    print('Horas en mower = '+str(datetime.timedelta(seconds=horaTotal)))
    print('__acabo__ mower')
    sys.stdout=stdoutOld
    mowerLog.close()
    #power down 
    call('shutdown -P',shell=True)
    
