#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  mower.py
#  
#  Copyright 2024  <rpi31@rpi31>
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
rato=0.1 #sec, 10Hz iaRTN limit
velStd=0.5 #m/s
velL=0 #/480
velR=0 #/480
#off time for leds
offShort=0.5 #sec
offMedium=1.0 #sec
offLong=3.0 #sec
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
ledError=gp.LED(21) #rojo
ledProcess=gp.LED(20) #amarillo, recordar o autonomo
ledGNSS=gp.LED(26) #azul
ledMotores=gp.LED(16) #verde
ledBeacon=gp.LED(19) #beacon's relay (blanco en pruebas)
btnRecordar=gp.Button(pin=27,pull_up=True)
btnCortar=gp.Button(pin=17,pull_up=True)
potLeft=gp.MCP3008(channel=3)
potRight=gp.MCP3008(channel=2)


def ctrlLuces(io): 
    #given requested bool state of all lights/sounds
    #returns nothing
    
    todoPins=(ledError,ledProcess,ledGNSS,ledMotores, ledBeacon)
    
    #luces, camera, accion
    #por todos
    for l in todoPins:
        if io:
            print('_ctrlLuces iniciando '+str(l.pin))
            # ~ l.on() #probando
            l.blink(on_time=0.1,off_time=0.1,n=1)
        else:
            print('_ctrlLuces apagando '+str(l.pin))
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
    
    # ~ try:
    # ~ print(f'numRollingAvg myList antes: {myList}')
    myList[myIndex]=myValue
    myIndex+=1
    if myIndex>len(myList)-1: myIndex=0
    rolledAvg=statistics.fmean(myList)
    # ~ print(f'numRollingAvg myList despues: {myList}')
    # ~ except:
        # ~ rolledAvg=None
    
    return myList, myIndex, rolledAvg

def cambiaControl(ratoSus=0):
    #given timeout for when all controls are at zero state
    # passes rheostat inputs to motor control. FORWARD ONLY
    # intended as an independent thread
    #returns nothing
    
    #inic
    print('_Iniciando cambiaControl con ratoSus='+str(ratoSus))
    timeCambia=time.time()
    ratoFin=False
    shutStart=False
    global velL, velR
    #suave
    histLeft=[0.001 for i in range(4)]
    histRight=[0.002 for i in range(4)]
    histIndexL=0
    histIndexR=0
    adcLeft=0
    adcRight=0
    potLeftBase=0.594
    potLeftArriba=0.826
    potRightBase=0.607
    potRightArriba=0.887
    mm=0
    mn=100
    avl=[]
    
    while (btnRecordar.is_pressed==True) and (not motoPara.is_set()) and (not ratoFin):
        print('__')
        #read and map pot switches to controller values
        adcLeft=potLeft.value
        adcRight=potRight.value
        # ~ tuneval=adcRight
        # ~ mn=min(mn,tuneval)
        # ~ mm=max(mm,tuneval)
        # ~ avl.append(tuneval)
        cmdLeft=(adcLeft-potLeftBase)/(potLeftArriba-potLeftBase)*480
        cmdRight=(adcRight-potRightBase)/(potRightArriba-potRightBase)*480
        histLeft,histIndexL,avgLeft=numRollingAvg(histLeft,histIndexL,cmdLeft)
        histRight,histIndexR,avgRight=numRollingAvg(histRight,histIndexR,cmdRight)
        #send inputs to controller
        velL,velR=motores.motCtrl(avgLeft,avgRight)
        print('adcLeft='+str(adcLeft))
        print('adcRight='+str(adcRight))
        print('avgs='+str([round(avgLeft,2),round(avgRight,2)]))
        print('vels='+str([velL, velR]))
                
        #throw timeout flag
        timeElapsed=time.time()-timeCambia
        print('ET='+str(round(timeElapsed,2)))
        if ((avgLeft<0.15*480) and (avgRight<0.20*480)):
            if (not shutStart) and (ratoSus>0):
                print(' Throttle shutdown bandera sacado en '+str(round(timeElapsed,1)))
                timeCambia=time.time()
                shutStart=True
        else:
            if shutStart and ratoSus>0:
                print(' Throttle shutdown bandera despejado en '+str(round(timeElapsed,1)))
                shutStart=False
        ratoFin=((ratoSus>0) and (time.time()-timeCambia>ratoSus) and shutStart)
        #don't overload anything
        time.sleep(rato)
        
    #report exits
    print('_Saliendo cambiaControl por')
    if not btnRecordar.is_pressed==True:
        print('  btnRecordar')
    if motoPara.is_set():
        print('  motoPara')
    if ratoFin:
        print('  ratoFin')
    
    # ~ print(f'min: {mn}')
    # ~ print(f'max: {mm}')
    # ~ mf=statistics.fmean(avl)
    # ~ print(f'avg: {mf}')
    # ~ mb=potLeftBase if tuneval==adcLeft else potRightBase
    # ~ ma=potLeftArriba if tuneval==adcLeft else potRightArriba
    # ~ print(f'alp: {(mf-mb)/(ma-mb)}')
    
    #shutdown
    ledMotores.blink(on_time=onTime,off_time=offShort,n=4)
    
    return

def enviarTicks():
    #regularly send wheel ticks (WT) over serial to GNSS
    #return nothing. Intended as independent thread
    
    global velL, velR
    
    rato1=1/10#s, GNSS receiving limit
    tickMult=(50*(2*3.14159/1)*(1/60)*(20/36)*(34.4/2))/0.01/480*rato1
    '''tick mult = scaled motor RPM to linear travel 
     = (max motor RPM*(2*pi rad / rev)*(min / 60 s)* _
      (gear pinion / gear rack)*(wheel dia cm/2))* _ 
      /100um/max command * time
     = approx. 1/480 
    '''
    
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
    #start led strip
    ctrlLuces(True) #todos
    time.sleep(offLong)
    ctrlLuces(False)
    #start engines
    motores.motInic()
    motThread=threading.Thread(target=cambiaControl, args=())
    motThread.start()
    #warm up GNSS
    gnss.gnssIniciar() #avoid wake wait?
    wtThread=threading.Thread(target=enviarTicks,args=())
    wtThread.start()
    
    #Choose the Mode
    #follow toggle orders
    horaVia=time.time
    archivoGuia=arcRuta+'rutaSeguida.txt'
    opcion=False
    
    #wait for gnss lock
    time.sleep(60)
    
    while horaVia-time.time<240 and not opcion:
        #read possible switches
        if btnRecordar.is_pressed: #record, or
            opcion=True
            #indicate lights
            ledProcess.blink(onTime, offShort)
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
        ledMotores.blink(onTime,offMedium)
        ledBeacon.blink(onTime,offShort,n=1)
        
    
    return sinErrores

def mowerPara(errStop):
    
    #cut engines
    motores.motPara()
    motoPara.set()
    global velL, velR
    velL=0
    velR=0
    #release GNSS
    gnss.gnssParar()
    #stop lights
    ctrlLuces(False)
    if errStop:
        mowerLog.close()
        ledBeacon.blink(on_time=onTime,off_time=offShort,n=60)
        ledError.blink(on_time=onTime,off_time=offShort,n=60)
        time.sleep(30)
        
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
    global velL, velR
    #velL=0
    #velR=0 #neg. forward
    pid=PID(1,0.1,0.05,setpoint=0)
    
    #GNSS locked
    ledGNSS.blink(onTime, offLong)
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
        velL,velR=cambiaGirar(pidR,velL,velR)
        
    
    #Sumer velocidad
    #ramp up wheel speed to 75% max or until intended position is reached
    lejoStart=4*radiusMatch
    spdPerc=0
    while goMow and spdPerc<=0.75 and lejoStart>2*radiusMatch:#m
        spdPerc+=0.1
        lejoStart=gc.numLejo(gnss.getPosition,datPointNext)
        velL,velR=cambiaGirar(spdPerc,velL,velR)
        time.sleep(0.1)
    #kick speed up if started too close to first point
    if velL<=0.25*480:
        velL=0.75*480
        velR=-velL
    
    
    #Iniciar cortar
    targetReached=False
    errPerd=0
    velCurr=velStd
    velAvg=[velCurr for i in range(100)]
    velAvgIndex=0
    timeSpdPrev=time.time
    #signal and start
    ledProcess.blink(onTime,offLong)
    for datPoint in datPointList:
        
        while goMow and not targetReached:
            
            #GNSS positioning
            gpsPointCurr=gnss.getPosition() #lat, long
            gpsPointChange=gc.numLejo(gpsPointPrev,gpsPointCurr) #m
            #continue after a reasonable change is detected
            if gpsPointChange>5/1000:
                #reset
                gpsPointPrev=gpsPointCurr #lat, long
                #continue
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
                        ledError.blink(on_time=onTime,off_time=flashMedium,n=10)
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
                        velL*=1.1
                        velR*=1.1
                        velL,velR=motores.motCtrl(velL,velR)
                    elif velAvgAvg>=1.5*velStd:
                        velL*=0.9
                        velR*=0.9
                        velL,velR=motores.motCtrl(velL,velR)
                    #input of heading differential (degree value amount needed)
                    pidR=pid(degChangeReq)
                    velL,velR=cambiaGirar(pidR,velL,velR)
                    
                    #reset
                    errPerd=0
                    targetReached=(distToNextPoint<=radiusMatch)
                
                else: #regard the point as a miss
                    errPerd+=1
                    targetReached=True
            
            else:
                time.sleep(rato/10)
            #end GNSS positioning
            
            if errPerd>5:
                ledError.on()
                print('!!! Muchos puntos perdidos en sequencia. Parandome')
                goMow=False
            
            if not btnCortar.is_pressed:
                ledProcess.blink(on_time=onTime,off_time=offShort)
                print('!!! Conexion a btnCortar perdido. Parandome')
                goMow=False
            
            #continua a punto
    #sigue con proximo punto
                
    #Cortar completa
    return goMow


def probar():
    global velL, velR
    while not motoPara.is_set():
        print('velL='+str(velL)+', velR='+str(velR))
        time.sleep(0.5)
        
    return


if __name__ == '__main__':
    print('__PROBANDO__ mower')
    
    print(datetime.date.today())
    # ~ pruebaThread=threading.Thread(target=probar,args=())
    # ~ pruebaThread.start()
    # ~ prPara=threading.Event()
    
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
    # ~ chkList=[i/10 for i in range(5)]
    # ~ k=0
    # ~ for j in chkList:
        # ~ chkList,k,a=numRollingAvg(chkList,k,j+1)
        # ~ print(chkList)
        # ~ print(a)
        
    # ~ print(os.path.dirname(os.path.abspath(__file__)))
    
    #probando luces y pot
    # ~ ledMotores.on()
    # ~ time.sleep(1)
    # ~ ctrlLuces(True)
    # ~ time.sleep(2)
    # ~ ctrlLuces(False)
    # ~ todoPins=(ledError,ledProcess,ledGNSS,ledMotores, ledBeacon)
    # ~ for l in todoPins:
        # ~ l.on()
        # ~ time.sleep(1)
        # ~ l.off()
    # ~ print(btnRecordar.is_pressed)
    # ~ print(btnCortar.is_pressed)
    # ~ time.sleep(1)
    
    #probando threading
    # ~ ledMotores.blink(on_time=.1,off_time=.1,n=4)
    # ~ time.sleep(1)
    # ~ ccT=threading.Thread(target=cambiaControl,args=([6]))
    # ~ ccT.start()
    # ~ lcT=threading.Thread(target=ctrlLuces,args=([True]))
    # ~ lcT.start()
    # ~ prT=threading.Thread(target=probar,args=())
    # ~ prT.start()
    # ~ time.sleep(2)
    # ~ ctrlLuces(False)
    # ~ time.sleep(3)
    # ~ motoPara.set()
    # ~ ccT.join()
    # ~ lcT.join()
    # ~ prT.join()
    
    #probando enviarTicks
    # ~ enviarTicks()
    
    #probando cambiaControl
    # ~ cambiaControl(10)
    
    # ~ prPara.set()
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
    
