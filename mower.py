#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  mower.py
#  
#  Copyright 2024  <rpi31@rpi31>
#  

#custom
import motores
# ~ import gnss
import gnss2
import geoCalc as gc
import fileMan
#standard
import time
import datetime
import gpiozero as gp
import os
import statistics
import threading
import subprocess as sc
#semi-custom
from simple_pid import PID
from pyubx2 import ubxtypes_core as ubt


#globals
arcRuta=os.path.dirname(os.path.abspath(__file__))+'/'#/home/rpi31/Documents/
radiusMatch=11*0.0254 #m, tamano de plataforma
rato=0.1 #sec, 10Hz iaRTN limit
velStd=0.5 #m/s
velL=0 #/480
velR=0 #/480
motoObj=motores.motoresObj()
gnaObj=None
#off time for leds
offShort=0.5 #sec
offMedium=1.0 #sec
offLong=3.0 #sec
onTime=0.5 #sec
#threading
motoPara=threading.Event()
gnssPara=threading.Event()
gnssFix=threading.Event()

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
# ~ ledError=gp.LED(21) #rojo, defined at end
# ~ ledProcess=gp.LED(20) #amarillo, recordar o autonomo
# ~ ledGNSS=gp.LED(26) #azul
# ~ ledMotores=gp.LED(16) #verde
ledBeacon=gp.LED(19) #beacon's relay (blanco en pruebas)
ledBeacon.off()
btnRecordar=gp.Button(pin=27,pull_up=True)
btnCortar=gp.Button(pin=17,pull_up=True)
potLeft=gp.MCP3008(channel=3)
potRight=gp.MCP3008(channel=2)

#errores tempranos
if motoObj.errores:
    mowerPara(True)


def ctrlLuces(rtk_Req): 
    #controla sobre la luz primera
    #dado la necessidad de un fix DGNSS/RTK
    '''
    #given requested bool state of all lights/sounds
    #returns nothing
    
    eliminado por conexiones feas
    
    todoPins=(ledError,ledProcess,ledGNSS,ledMotores, ledBeacon)
    
    #luces, camera, accion
    #por todos
    for l in todoPins:
        if io:
            print('_ctrlLuces iniciando '+str(l.pin))
            # ~ l.on() #probando
            l.blink(on_time=offLong,off_time=offLong,n=1)
            time.sleep(offMedium)
        else:
            print('_ctrlLuces apagando '+str(l.pin))
            l.off()
    '''
    
    # Avisar al usario por el GNSS Fix
    horaEntrada=time.time()
    ledBeacon.blink(on_time=onTime,off_time=offLong)
    #TTF3dF
    fT=0
    while ((not gnssPara.is_set()) and (fT<3)):
        try:
            fT=gnaObj.fixType
        except:
            pass
        time.sleep(1)
    horaTTF=time.time()
    print(f'ctrlLuces   Rato por primera 3D fix: {round(horaTTF-horaEntrada,1)}s')
    #si necesita un fix de mas alta calidad
    if rtk_Req:
        ledBeacon.blink(on_time=onTime,off_time=offMedium)
        while ((not gnssPara.is_set()) and fT!=4):
            try:
                fT=gnaObj.fixType
            except:
                pass
            time.sleep(1)
        print(f'ctrlLuces  Rato por alta 3D fix: {time.time()-horaTTF}s')
    #Avisar por encender el motor
    horaInic1=time.time()+15#s
    ledBeacon.blink(on_time=onTime,off_time=offShort,n=int(onTime+offShort)/15)
    while ((not gnssPara.is_set()) and (time.time()<horaInic1)):
        time.sleep(rato)
    #Iniciar
    if not gnssPara.is_set():
        print('ctrlLuces    Ojala que el motor ha sido encendido...')
        ledBeacon.blink(on_time=0.1,off_time=0.1,n=10)
        time.sleep(1)
    
    ledBeacon.off()
    
    gnssFix.set()
    
    print(f'ctrlLuces    Luces listos en {time.time()-horaEntrada}s')
    
    return

def cambiaGirar(percIn,velL,velR):
    '''
    given a unity input (percentage of maximum velocity)
        scales value to motor board's preferred map
        positive required differential
         = positive percIn --> CCW spin of mower
    echoes commanded velocities per motor
    '''
    
    delRueda=-motoObj.velMax*percIn
    velL=max(-motoObj.velMax,min(motoObj.vel1act+delRueda,motoObj.velMax))
    velR=max(-motoObj.velMax,min(motoObj.vel2act+delRueda,motoObj.velMax))
    motoObj.vel(velL,velR)
    
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

def cambiaControl(ratoSus=0, prueba=False):
    #given timeout for when all controls are at zero state
    # passes rheostat inputs to motor control. FORWARD ONLY
    # intended as an independent thread
    #returns nothing
    
    #inic
    motoObj.prueba=prueba
    print('mower    Iniciando cambiaControl con ratoSus='+str(ratoSus))
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
    #tuned
    potLeftBase=0.594
    potLeftArriba=0.826
    potRightBase=0.607
    potRightArriba=0.887
    #tuners
    mm=0
    mn=100
    avl=[]
    
    while (btnRecordar.is_pressed==True) and (not motoPara.is_set()) and (not ratoFin):
        if prueba:
            print('mower    __')
        #read and map pot switches to controller values
        adcLeft=potLeft.value
        adcRight=potRight.value
        if prueba:
            tuneval=adcRight
            mn=min(mn,tuneval)
            mm=max(mm,tuneval)
            avl.append(tuneval)
        cmdLeft=(adcLeft-potLeftBase)/(potLeftArriba-potLeftBase)*motoObj.velMax
        cmdRight=(adcRight-potRightBase)/(potRightArriba-potRightBase)*motoObj.velMax
        histLeft,histIndexL,avgLeft=numRollingAvg(histLeft,histIndexL,cmdLeft)
        histRight,histIndexR,avgRight=numRollingAvg(histRight,histIndexR,cmdRight)
        
        #send inputs to controller (forward only)
        velL,velR=motoObj.vel(avgLeft,-avgRight)
        if prueba:
            print('mower    adcLeft='+str(adcLeft))
            print('mower    adcRight='+str(adcRight))
            print('mower    avgs='+str([round(avgLeft,2),round(avgRight,2)]))
            print('mower    vels='+str([velL, velR]))
                
        #throw timeout flag
        timeElapsed=time.time()-timeCambia
        if prueba:
            print('mower    ET='+str(round(timeElapsed,2)))
        if ((avgLeft<0.15*480) and (avgRight<0.20*480)):
            if (not shutStart) and (ratoSus>0):
                print('mower     Throttle shutdown bandera sacado en '+str(round(timeElapsed,1)))
                timeCambia=time.time()
                shutStart=True
        else:
            if shutStart and ratoSus>0:
                print('mower     Throttle shutdown bandera despejado en '+str(round(timeElapsed,1)))
                shutStart=False
        ratoFin=((ratoSus>0) and (time.time()-timeCambia>ratoSus) and shutStart)
        
        #don't overload anything
        time.sleep(rato)
        
    #report exits
    dt=''
    if not btnRecordar.is_pressed==True:
        dt+=' btnRecordar'
    if motoPara.is_set():
        dt+=' motoPara'
    if ratoFin:
        dt+=' ratoFin'
    print(f'mower    Saliendo cambiaControl por{dt}')
    
    if prueba:
        print(f'mower    min: {mn}')
        print(f'mower    max: {mm}')
        mf=statistics.fmean(avl)
        print(f'mower    avg: {mf}')
        mb=potLeftBase if tuneval==adcLeft else potRightBase
        ma=potLeftArriba if tuneval==adcLeft else potRightArriba
        print(f'mower    alp: {(mf-mb)/(ma-mb)}')
    
    #shutdown
    # ~ ledMotores.blink(on_time=onTime,off_time=offShort,n=4)
    
    return

def enviarTicks():
    #regularly send wheel ticks (WT) over serial to GNSS
    #return nothing. Intended as an independent thread
    
    global motoObj
    global gnaObj
    
    rato1=1/10#s, GNSS receiving limit
    
    #define tick constructors
    maxVel=50*(2*3.14159/1)*(1/60)*(20/36)*(0.344/2)#m/s, ~0.50m/s
    '''   = max motor RPM*(2*pi rad / rev)*(min / 60 s)
            * (gear pinion / gear rack)*(wheel dia/2)
    '''
    tickScale=round(maxVel/480*rato1,6)#m/tick ~=105um
    tickMult=maxVel*rato1/tickScale/480 #ticks/cmd
    '''tick mult = scaled motor RPM to linear travel by time
      = max speed * time / dist. per tick / max command
      ~= 1
    '''
    
    def _enviar(velL,velR):
        #normal forward = positive
        wtL=abs(round(velL*tickMult,0))
        wdL=(velL>=0)
        #normal forward = negative
        wtR=abs(round(velR*tickMult,0))
        wdR=(velR<0)
        
        #gnss.sendWT(wtL,wtR)
        tsrc=datetime.datetime.now()
        ttag=int(tsrc.microsecond/1000)
        gnaObj.put_wheelTick(wtL,wdL,wtR,wdR,ttag)
    
    #integrals-ish
    subdivide=(motoObj._rato<rato1)
    if subdivide:
        rollLen=int(rato1/motoObj._rato)
        rollListL=[0.0 for i in range(rollLen)]
        rollIndexL=0
        rollAvgL=0
        rollListR=[0.0 for i in range(rollLen)]
        rollIndexR=0
        rollAvgR=0
    
    #give board a heads-up
    gnaObj.cfg_WT(1,int(tickScale*pow(10,6)))
    
    if subdivide:
        t0=datetime.datetime.now()
        while not motoPara.is_set():
            t1=datetime.datetime.now()
            t1_t0=t1-t0
            while t1_t0.microseconds*pow(10,6)<rato1:
                rollListL,rollIndexL,rollAvgL=numRollingAvg(rollListL,rollIndexL,motoObj.vel1Act)
                rollListR,rollIndexR,rollAvgR=numRollingAvg(rollListR,rollIndexR,motoObj.vel2Act)
                time.sleep(motoObj._rato)
                t1_t0=datetime.datetime.now()-t0
            t0=datetime.datetime.now()
            _enviar(int(rollAvgL),int(rollAvgR))
    else:
        while not motoPara.is_set():
            _enviar(motoObj.vel1Act,motoObj.vel2Act)
            time.sleep(rato1)
    return

def mowerInic():
    #startup sequence for any mowing op
    #returns general status
    
    sinErrores=True
    
    global gnaObj
    
    #Startup and Signaling
    '''
    #start led strip
    ctrlLuces(True) #todos
    time.sleep(offLong)
    ctrlLuces(False)
    '''
    #start engines
    motThread=threading.Thread(target=cambiaControl, args=())
    motThread.start()
    #warm up GNSS
    # ~ gnss.gnssIniciar()
    gnaObj, sendQ = gnss2.gnssIniciar(gnssPara)
    wtThread=threading.Thread(target=enviarTicks,args=())
    wtThread.start()
    
    #Choose the Mode
    #follow toggle orders
    horaVia=time.time()
    archivoGuia=arcRuta+'rutaSeguida.txt'
    opcion=False
    
    while horaVia-time.time()<240 and not opcion:
        #read possible switches
        if btnRecordar.is_pressed: #record, or
            opcion=True
            #indicate lights
            # ~ ledProcess.blink(onTime, offShort)
            ctrlLuces(True)
            #record location of guide
            sinErrores, archivoNuevo=recordar()
            recArchivo=open(archivoGuia,'w')
            recArchivo.write(archivoNuevo)
            recArchivo.close()
            
        elif btnCortar.is_pressed: #cut
            opcion=True
            motoPara.set()
            #indicate lights
            # ~ ledProcess.on()
            ctrlLuces(False)
            #get most recent point list
            with open(archivoGuia,'r') as arcAct:
                archivoActual=arcAct.readline()
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
        # ~ gnss.gnssParar()
        gnssPara.set()
        time.sleep(1)
        gnaObj.stop()
        # ~ ledProcess.off()
        # ~ ledMotores.blink(onTime,offMedium)
        ledBeacon.off()
        
    
    return sinErrores

def mowerPara(errStop):
    
    #cut engines
    if errStop:
        motoObj.motPara()
    else:
        motoObj.paraSuave()
    motoPara.set()
    global velL, velR
    velL=0
    velR=0
    #release GNSS
    # ~ gnss.gnssParar()
    gnssPara.set()
    gnaObj.stop()
    
    #stop lights
    # ~ ctrlLuces(False)
    ledError=gp.LED(21)
    if errStop:
        mowerLog.close()
        ledBeacon.blink(on_time=onTime,off_time=offShort,n=60)
        ledError.blink(on_time=onTime,off_time=offShort,n=60)
        time.sleep(30)
    else:
        ledError.on()
    #release & reinstate GPIO
    ledBeacon.close()
    print(sc.run(["pinctrl set 19 pd"],shell=True,text=True,check=True))
    ledError.close()
    print(sc.run(["pinctrl set 21 pd"],shell=True,text=True,check=True))
    
    return

def recordar(prlist=[]):
    #record set of coordinates as a list to a local file
    #returns general status and full path of file made
    
    hazRec=True
    
    #prep file for recording
    archivoRuta=arcRuta+'rutas/'
    archivoFecha=str(datetime.date.today())
    #check and set for any copy names
    archivoRuta=fileMan.prefixMatch(archivoRuta,archivoFecha)+'.txt'
    
    #record points to follow
    with open(archivoRuta,'w') as a:
        ddV=[0.1,0.1]
        #wait for button debounce
        waits=0
        while btnRecordar.is_pressed==False:
            time.sleep(rato)
            waits+=1
            if waits>10:
                print('mower    No pude recordar sin btnRecordar. Saliendo')
                break
        # ~ for dd in prlist: #testing
        while btnRecordar.is_pressed==True:
            time.sleep(rato)
            # ~ dd=gnss.getPosition()
            dd=gnaObj.get_position()
            #skip very close recordings
            larga=gc.numLejo(dd,ddV)#m
            if larga>=0.05:
                datu=' '.join(str(ll) for ll in dd)
                a.write(datu+'\n')
                ddV=dd
        
    #complete
    motoPara.set() #backup
    motThread.join()
    # ~ ledProcess.on()
    
    
    return hazRec, archivoRuta

def cortar(datPointList):
    #follows given coordinate list
    #returns general status
    
    #Scope init
    goMow=True
    #ruedas
    global velL, velR
    #velL=0
    #velR=0 #neg. forward
    pidDeg=PID(0.03,0.0001,0.0001,setpoint=0)
    pidDeg.sample_time=rato
    pidDeg.output_limits=(-1,1)
    
    #GNSS locked
    # ~ ledGNSS.blink(onTime, offLong)
    ledBeacon.blink(on_time=onTime, off_time=offMedium)
    
    #First spin to first point
    headDel=180
    #get intended positions
    datPointNext=datPointList[0] #lat, long
    td0=time.time()
    while goMow and (abs(headDel)>5):
        #get current position
        # ~ gpsPointPrev=gnss.getPosition() #brujula degrados: lat, long
        gpsPointPrev=gnaObj.get_position()
        #determine required heading to intended position
        rumPrimera=gc.numRumbo(gpsPointPrev,datPointNext) #deg
        # ~ gpsHead=gnss.getHeading() #deg
        gpsHead=gnaObj.heading()
        headDel=gc.numGirar(gpsHead,rumPrimera) #deg
        #rotate to heading
        pidR=pidDeg(headDel)
        velL,velR=cambiaGirar(pidR,velL,velR)
        time.sleep(pidDeg.sample_time)
        #timeout
        if (time.time()-td0<20):
            print('cortar   !!!  Algo feo occurio en rumbando. Parandome')
            goMow=False
        
    
    #Sumer velocidad
    #ramp up wheel speed to 75% max or until intended position is reached
    lejoStart=4*radiusMatch
    spdPerc=0
    while goMow and spdPerc<=0.75 and lejoStart>2*radiusMatch:#m
        spdPerc+=0.1
        # ~ lejoStart=gc.numLejo(gnss.getPosition(),datPointNext)
        lejoStart=gc.numLejo(gnaObj.get_position(),datPointNext)
        velL,vel=motoObj.vel(spdPerc*velL,spdPerc*velR)
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
    timeSpdPrev=time.time()
    headListIMU=[gpsHead+1 for i in range(12)]
    headIndexIMU=0
    headListPoints=[gpsHead-1 for i in range(12)]
    headIndexPoints=0
    #signal and start
    # ~ ledProcess.blink(onTime,offLong)
    for datPoint in datPointList:
        
        while goMow and not targetReached:
            
            #GNSS positioning
            # ~ gpsPointCurr=gnss.getPosition() #lat, long
            gpsPointCurr=gnaObj.get_position()
            gpsPointChange=gc.numLejo(gpsPointPrev,gpsPointCurr) #m
            #continue after a reasonable change is detected
            if gpsPointChange>0.025:
                #reset
                gpsPointPrev=gpsPointCurr #lat, long
                #continue
                distToNextPoint=gc.numLejo(gpsPointCurr,datPoint) #m
                degHeadingIntend=gc.numRumbo(gpsPointCurr,datPoint) #brujula degrees
                
                #Continue if point is in front of mower
                if degHeadingIntend<110:
                    timeSpdCurr=time.time() #for later
                    
                    #GNSS headings
                    #get gnss given heading
                    # ~ degHeadingGNSS=gnss.getHeading() #brujula degrees
                    degHeadingGNSS=gnaObj.heading()
                    headListIMU, headIndexIMU, degHeadGNSSAvg = numRollingAvg(headListIMU,headIndexIMU,degHeadingGNSS)
                    #determine actual heading
                    degHeadingActual=gc.numRumbo(gpsPointPrev,gpsPointCurr) #deg
                    headListPoints,headIndexPoints,degHeadActualAvg=numRollingAvg(headListPoints,headIndexPoints,degHeadingActual)
                    if abs(numGirar(degHeadActualAvg,degHeadGNSSAvg))>30:
                        # ~ ledError.blink(on_time=onTime,off_time=flashMedium,n=10)
                        print('mower    !!! GNSS que necesita ajuste, o giramos rapidamente.')
                        print('mower         Rumbo observado: '+str(round(degHeadGNSSAvg)))
                        print('mower         Rumbo calculado: '+str(round(degHeadActualAvg)))
                    #give shortest spin angle to heading
                    degChangeReq=gc.numGirar(degHeadActualAvg,degHeadingIntend) #deg
                                
                    #Twist the wheels
                    #check centerline speed to standard
                    velCurr=gc.numLejo(gpsPointPrev,gpsPointCurr)/(timeSpdCurr-timeSpdPrev)#m/s
                    timeSpdPrev=time.time()
                    velAvg,velAvgIndex,velAvgAvg=numRollingAvg(velAvg,velAvgIndex,velCurr)
                    if velAvgAvg<velStd:
                        velL*=1.1
                        velR*=1.1
                        velL,velR=motoObj.vel(velL,velR)
                    elif velAvgAvg>=1.2*velStd:
                        velL*=0.9
                        velR*=0.9
                        velL,velR=motoObj.vel(velL,velR)
                    #input of heading differential (degree value amount needed)
                    pidR=pidDeg(degChangeReq)
                    velL,velR=cambiaGirar(pidR,velL,velR)
                    
                    #reset
                    errPerd=0
                    errMovi=0
                    targetReached=(distToNextPoint<=radiusMatch)
                
                else: #regard the point as a miss
                    errPerd+=1
                    targetReached=True
            
            else:
                time.sleep(rato/10)
                errMovi+=1
                if errMovi>4/(rato/10):
                    print('cortar   !!! Estoy atascado. Auxilio, porfa.')
                    goMow=False
            #end GNSS positioning
            
            if errPerd>5:
                # ~ ledError.on()
                print('cortar   !!! Muchos puntos perdidos en sequencia. Parandome')
                goMow=False
            
            if not btnCortar.is_pressed:
                # ~ ledProcess.blink(on_time=onTime,off_time=offShort)
                print('cortar   !!! Conexion a btnCortar perdido. Parandome')
                goMow=False
            
            #continua a punto
    #sigue con proximo punto
                
    #Cortar completa
    return goMow


if __name__ == '__main__':
    print('__PROBANDO__ mower')
    
    print(datetime.date.today())
    #print(datetime.datetime.now())
    
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
        
    #probando luces y pot
    # ~ ledBeacon.blink(on_time=onTime,off_time=offShort,n=10)
    # ~ time.sleep(10)
    # ~ ctrlLuces(True)
    # ~ time.sleep(2)
    # ~ ctrlLuces(False)
    # ~ print(btnRecordar.is_pressed)
    # ~ print(btnCortar.is_pressed)
    
    #probando threading
    # ~ ccT=threading.Thread(target=cambiaControl,args=([6]))
    # ~ ccT.start()
    # ~ lcT=threading.Thread(target=ctrlLuces,args=([True]))
    # ~ lcT.start()
    # ~ time.sleep(6)
    # ~ ctrlLuces(False)
    # ~ time.sleep(1)
    # ~ motoPara.set()
    # ~ ccT.join()
    # ~ lcT.join()
    
    #probando enviarTicks
    # ~ enviarTicks()
    
    #probando cambiaControl
    # ~ cambiaControl(10,True)
    
    #probando motores
    print('motos incendido')
    motoObj.prueba=True
    for y in range(50,360,10):
        print(y)
        velL, velR=motoObj.vel(y,-y)
        time.sleep(0.02)
    motoObj.motPara()
    print('motos apagado')
    
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
    hora0=time.time()
    sinErrores=mowerInic()
    mowerPara(not sinErrores)
    horaTotal=time.time-hora0
    print(f'Horas en mower = {datetime.timedelta(seconds=horaTotal)}')
    print('__acabo__ mower')
    sys.stdout=stdoutOld
    mowerLog.close()
    #power down 
    sc.run(['shutdown -P'], shell=True, text=True, check=True)
    
