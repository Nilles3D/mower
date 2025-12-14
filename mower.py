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
# ~ import gnss2
import gnss3 as gnss
import geoCalc as gc
import fileMan
import btApp
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
arcRuta=os.path.dirname(os.path.abspath(__file__))+'/'
radiusMatch=11*0.0254 #m, tamano de plataforma
rato=0.1 #sec, 10Hz iaRTN limit
velStd=0.8*0.5 #m/s
velL=0 #/480
velR=0 #/480
gnaObj=None
mowerLog=None
#off time for leds
offShort=0.5 #sec
offMedium=1.0 #sec
offLong=3.0 #sec
onTime=0.5 #sec
#threading
motoPara=threading.Event()
gnssPara=threading.Event()
gnssFix=threading.Event()
manualShut=threading.Event()

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



def ctrlLuces(rtk_Req): 
    #controla sobre la luz primera
    #dado la necessidad de un fix DGNSS/RTK
    #devuelva si hubiera errores o no
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
    cond = 'si' if rtk_Req else 'no'
    print(f'ctrlLuces   Un fix RTK {cond} es requido')
    
    sinErr=True
    
    # Avisar al usario por el GNSS Fix
    horaEntrada=time.time()
    ledBeacon.blink(on_time=onTime,off_time=offLong)
    #TTF3dF
    fT=0
    fT0=0
    gps0=gnaObj.get_position()
    print('ctrlLuces    Esperando por un 3D Fix')
    while ((not gnssPara.is_set()) and (fT<0)): #should be ft<3
        try:
            fT=gnaObj.fixType
            gps1=gnaObj.get_position()
            if gps0!=gps1:
                print(f'ctrlLuces   GPS = {gps1}')
                gps0=gps1
        except:
            pass
        if fT!=fT0:
            print(f'ctrlLuces   FixType = {fT}')
            fT0=fT
        time.sleep(1)
        if time.time()-horaEntrada>120:
            print('ctrlLuces    Excesso de tiempo en esperar por un Fix. gnssPara set')
            gnssPara.set()
            sinErr=False
    horaTTF=time.time()
    print(f'ctrlLuces   Rato por primera 3D fix: {round(horaTTF-horaEntrada,1)}s')
    #si necesita un fix de mas alta calidad
    if rtk_Req and sinErr:
        ledBeacon.blink(on_time=onTime,off_time=offMedium)
        print('ctrlLuces    Esperando por un 3D Fix de alta calidad')
        while ((not gnssPara.is_set()) and fT!=4):
            try:
                fT=gnaObj.fixType
                gps2=gnaObj.get_position()
                if gps1!=gps2:
                    print(f'ctrlLuces   GPS = {gps2}')
                    gps1=gps2
            except:
                pass
            time.sleep(1)
            if time.time()-horaEntrada>240:
                print('ctrlLuces    Excesso en esperar por un Fix Alta. gnssPara set')
                gnssPara.set()
                sinErr=False
        print(f'ctrlLuces  Rato por alta 3D fix: {time.time()-horaTTF}s')
    #Avisar por encender el motor
    horaInic1=time.time()+15#s
    nB=int((onTime+offShort)/15)
    ledBeacon.blink(on_time=onTime,off_time=offShort,n=nB)
    print('ctrlLuces    On your marks, get set, ...')
    while ((not gnssPara.is_set()) and (time.time()<horaInic1)) and sinErr:
        time.sleep(rato)
    #Iniciar
    if (not gnssPara.is_set()) and sinErr:
        print('ctrlLuces    Ojala que el motor ha sido encendido...')
        ledBeacon.blink(on_time=0.1,off_time=0.1,n=10)
        time.sleep(1)
    
    ledBeacon.off()
    
    gnssFix.set()
    
    print(f'ctrlLuces    Luces listos en {round(time.time()-horaEntrada,0)}s')
    
    return sinErr

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
    print('mower    Iniciando cambiaControl con ratoSus='+str(ratoSus))
    timeCambia=time.time()
    ratoFin=False
    shutRequest=False
    shutStart=False
    global velL, velR
    #suave
    histLeft=[0.001 for i in range(6)]
    histRight=[0.002 for i in range(6)]
    histIndexL=0
    histIndexR=0
    adcLeft=0
    adcRight=0
    avgLeft=1
    avgRight=1
    #tuned
    potLeftBase=0.085
    potLeftArriba=0.353
    potRightBase=0.128
    potRightArriba=0.290
    #tuners
    mm=[0,0]
    mn=[100,100]
    avl=[]
    avr=[]
    
    while (not motoPara.is_set()) and (not ratoFin):
        if prueba:
            print('mower    __')
        #get remote values
        if btApp.btReady: #remote control
            strLeft=min(max(-100,100+btApp.steerVal),100)/100
            strRight=min(max(-100,100-btApp.steerVal),100)/100
            pwrLvl=min(max(-100,btApp.powerVal),100)/100
            cmdLeft=pwrLvl*strLeft*motoObj.velMax
            cmdRight=pwrLvl*strRight*motoObj.velMax
        #read and average pot switches
        else: #manual sticks
            adcLeft=potLeft.value
            adcRight=potRight.value
            if prueba:
                tuneval=[adcLeft,adcRight]
                for v in range(0,len(tuneval)):
                    mn[v]=min(mn[v],tuneval[v])
                    mm[v]=max(mm[v],tuneval[v])
                avl.append(tuneval[0])
                avr.append(tuneval[1])
            histLeft,histIndexL,avgLeft=numRollingAvg(histLeft,histIndexL,adcLeft)
            histRight,histIndexR,avgRight=numRollingAvg(histRight,histIndexR,adcRight)
            #map to controller values
            cmdLeft=(avgLeft-potLeftBase)/(potLeftArriba-potLeftBase)*motoObj.velMax
            cmdRight=(avgRight-potRightBase)/(potRightArriba-potRightBase)*motoObj.velMax
        
        #send inputs to controller (forward only)
        velL,velR=motoObj.vel(cmdLeft,-cmdRight)
        if prueba:
            print(f"mower    btApp.btReady = {btApp.btReady}")
            print(f"mower    btApp.powerVal = {btApp.powerVal}")
            print(f"mower    btApp.steerVal = {btApp.steerVal}")
            print('mower    adcLeft='+str(adcLeft))
            print('mower    adcRight='+str(adcRight))
            print('mower    avgs='+str([round(avgLeft,2),round(avgRight,2)]))
            print('mower    vels='+str([velL, velR]))
                
        #throw timeout flag
        timeElapsed=time.time()-timeCambia
        if prueba:
            # ~ shutStart=True
            print('mower    ET='+str(round(timeElapsed,2)))
            print(f"mower    ratoSus = {ratoSus}")
        #seek requests for shutdown
        if (btApp.btReady and abs(btApp.powerVal)<=5): #priority to remote control
            shutRequest=True
        elif((not btApp.btReady) and ((abs(avgLeft)<1.015*potLeftBase) and (abs(avgRight)<1.020*potRightBase))):
            shutRequest=True
        else:
            shutRequest=False
        #act on said requests
        if shutRequest:
            if (not shutStart) and (ratoSus>0):
                print('mower     Throttle shutdown bandera sacado en '+str(round(timeElapsed,1)))
                timeCambia=time.time()
                shutStart=True
        else:
            if shutStart and ratoSus>0:
                print('mower     Throttle shutdown bandera despejado en '+str(round(timeElapsed,1)))
                shutStart=False
        #late test check
        if prueba:
            print(f"mower    shutStart = {shutStart}")
            
        ratoFin=((ratoSus>0) and (time.time()-timeCambia>ratoSus) and shutStart)
        
        #don't overload anything
        time.sleep(0.1*rato)
        
    #report exits
    dt=''
    if motoPara.is_set():
        dt+=' motoPara'
    if ratoFin:
        dt+=' ratoFin'
    print(f'mower    Saliendo cambiaControl por{dt}')
    
    if prueba:
        print(f'mower          L     R')
        print(f'mower    min: {round(mn[0],3):04.3f}, {round(mn[1],3):04.3f}')
        print(f'mower    max: {round(mm[0],3):04.3f}, {round(mm[1],3):04.3f}')
        mfl=statistics.fmean(avl)
        mfr=statistics.fmean(avr)
        print(f'mower    avg: {round(mfl,3):04.3f}, {round(mfr,3):04.3f}')
        print(f'mower    alp: {round((mfl-potLeftBase)/(potLeftArriba-potLeftBase),2):.0%}, {round((mfr-potRightBase)/(potRightArriba-potRightBase),2):.0%}')
    
    #shutdown
    # ~ ledMotores.blink(on_time=onTime,off_time=offShort,n=4)
    manualShut.set()
    
    
    return

def enviarTicks():
    #UNCONFIRMED. DO NOT USE.
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
    print(f'enviarTicks  WT seran enviado con scale = {round(tickScale,0)} y mult = {round(tickMult,1)}')
    
    def _enviar(velL,velR):
        #normal forward = positive
        wtL=int(abs(round(velL*tickMult,0)))
        wdL=(velL>=0)
        #normal forward = negative
        wtR=int(abs(round(velR*tickMult,0)))
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
        print('enviarTicks progresando con integrals')
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
        print('enviarTicks  progresando con pasos simples y largos')
        while not motoPara.is_set():
            _enviar(motoObj.vel1Act,motoObj.vel2Act)
            time.sleep(rato1)
    
    print('enviarTicks  fin')
    
    return

def logSave(fileObj, stopEvent):
    print('logSave start')
    timeDelt=0.0
    hora01=0.0
    while not stopEvent.is_set():
        if timeDelt>5.0:
            fileObj.flush()
            os.fsync(fileObj.fileno())
            hora01=time.time()
        timeDelt=time.time()-hora01
    return
    
def mowerInic():
    #startup sequence for any mowing op
    #returns general status
    
    #errores tempranos
    if motoObj.errores:
        mowerPara(True)
        return False
    
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
    motThread.start()
    btThread.start()
    #warm up GNSS
    gnaObj = gnss.gnssIniciar(gnssPara)
    # ~ wtThread.start()
    
    #Choose the Mode
    #follow toggle orders
    horaVia=time.time()
    horaEn=0
    horaLim=240
    archivoGuia=arcRuta+'rutaSeguida.txt'
    opcion=False
    
    print('mower    Esperando por eligiones humanos...')
    ledBeacon.blink(on_time=onTime, off_time=offLong)
    while horaEn<horaLim and not opcion:
        #read possible switches
        if btnRecordar.is_pressed: #record, or
            opcion=True
            print('mowerInic    btnRecordar eligido')
            #indicate lights
            # ~ ledProcess.blink(onTime, offShort)
            if ctrlLuces(True):
                #record location of guide
                sinErrores, archivoNuevo=recordar()
                recArchivo=open(archivoGuia,'w')
                recArchivo.write(archivoNuevo)
                recArchivo.close()
            else:
                sinErrores=False
            
        elif btnCortar.is_pressed: #cut
            opcion=True
            print('mowerInic    btnCortar eligido')
            motoPara.set()
            btApp.cerrar()
            #indicate lights
            # ~ ledProcess.on()
            if ctrlLuces(False):
                #get most recent point list
                with open(archivoGuia,'r') as arcAct:
                    archivoActual=arcAct.readline()
                datPointList=[]
                print(f'mowerInic   progresando con {archivoActual}')
                #read in the list
                with open(archivoActual,'r') as arcSrc:
                    for linea in arcSrc:
                        d=linea[:-1]
                        l=d.split()
                        ll=[float(x) for x in l]
                        datPointList.append(ll)
                #cut
                print(f'mowerInic   siguiendo {len(datPointList)} puntos en linea')
                sinErrores=cortar(datPointList)
            else:
                sinErrores=False
        
        else: #wait for human to start
            time.sleep(1)
        
        horaEn=int(round(time.time()-horaVia,0))
        if horaEn % 10 == 0:
            print(f'mowerInic    {horaEn} de {horaLim} segundos')
    
    if not opcion: #manual operation only
        print('mowerInic    Manual eligido')
        # ~ gnss.gnssParar()
        gnssPara.set()
        time.sleep(2)
        gnaObj.stop()
        # ~ ledProcess.off()
        # ~ ledMotores.blink(onTime,offMedium)
        ledBeacon.off()
        print('mowerInic    Esperando por manualShut')
        manualShut.wait()
        
    print('mowerInic   fin')
    
    return sinErrores

def mowerPara(errStop):
    
    print(f'mowerPara   Detente solicitado con emergencia = {errStop}')
    
    #cut engines
    if errStop:
        motoObj.motPara()
    else:
        motoObj.paraSuave()
    print('mowerPara    Motores han sido apagados')
    if motoObj.mf1>=100 or motoObj.mf2>=100:
        errStop=True
    motoPara.set()
    print('mowerPara    Controles manuales apagando')
    if btApp.btReady:
        btApp.cerrar()
    print('mowerPara    BT conexion ha sido apagado')
    global velL, velR
    velL=0
    velR=0
    #release GNSS
    # ~ gnss.gnssParar()
    gnssPara.set()
    time.sleep(2)
    print('mowerPara    GNSS para solicitado')
    if not gnaObj is None:
        gnaObj.stop()
    print('mowerPara    GNSS han sido apagado')
    
    #stop lights
    # ~ ctrlLuces(False)
    ledError=gp.LED(21)
    if errStop:
        if not mowerLog is None:
            mowerLog.close()
        ledBeacon.blink(on_time=onTime,off_time=offShort,n=60)
        ledError.blink(on_time=onTime,off_time=offShort,n=60)
        time.sleep(30)
    else:
        ledError.on()
    #release & reinstate GPIO
    ledBeacon.close()
    print('mowerPara    Luces han sido apagados')
    print(sc.run(["pinctrl set 19 pd"],shell=True,text=True,check=True))
    ledError.close()
    print(sc.run(["pinctrl set 21 pd"],shell=True,text=True,check=True))
    
    print('mowerPara    fin')
    
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
    
    print(f'recordar    Recordando en {archivoRuta}')
    
    #record points to follow
    with open(archivoRuta,'w') as a:
        ddV=[0.1,0.1]
        #wait for button debounce
        waits=0
        while btnRecordar.is_pressed==False and len(prlist)<=0:
            time.sleep(rato)
            waits+=1
            if waits>10:
                print('recordar    No pude recordar sin btnRecordar. Saliendo')
                break
        # ~ for dd in prlist: #testing
        while btnRecordar.is_pressed==True:
            time.sleep(rato)
            #dd=gnss.getPosition()
            dd=gnaObj.get_position()
            #skip very close recordings
            larga=gc.numLejo(dd,ddV)#m
            if larga>=0.05:
                datu=' '.join(str(ll) for ll in dd)
                a.write(datu+'\n')
                ddV=dd
    
    print('recordar recordar completo')
    
    #complete
    motoPara.set() #backup
    if motThread.is_alive():
        motThread.join()
    # ~ ledProcess.on()
    
    print('recordar fin')
    
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
    #get current position
    gpsPointPrev=gnaObj.get_position()
    #get intended positions
    datPointNext=datPointList[0] #lat, long
    td0=time.time()
    #init backup heading finders
    tJog=td0
    jogNow=True
    gpsHead=0
    #spin to win
    print(f'cortar   Iniciando desde {gpsPointPrev}')
    while goMow and (abs(headDel)>5):
        #determine required heading to intended position
        rumPrimera=gc.numRumbo(gpsPointPrev,datPointNext) #deg
        # ~ gpsHead=gnss.getHeading() #deg
        if gnaObj.heading==0: #bad sensor reading
            if jogNow:
                #jog the wheels
                print('cortar   Voy a intentar un jog')
                velL, velR=motoObj.vel(160,-160)
                time.sleep(2)
                #eval
                gpsPointJog=gnaObj.get_position()
                if gc.numLejo(gpsPointPrev,gpsPointJog)<0.05:
                    print(f'cortar  cero desde cero, {gpsPointJog}')
                else:
                    print(f'cortar  de {gpsPointPrev} hasta {gpsPointJog}')
                    gpsHead=gc.numRumbo(gpsPointPrev,gpsPointJog)
                    print(f'cortar  Aproximamente {round(gpsHead,0)} deg')
                    gpsPointPrev=gpsPointJog
                    jogNow=False
                #jog performed or needs to be performed again
                tJog=time.time()
        else:
            gpsHead=gnaObj.heading
            gpsPointPrev=gnaObj.get_position()
        headDel=gc.numGirar(gpsHead,rumPrimera) #deg
        #rotate to heading
        pidR=pidDeg(headDel)
        # ~ velL,velR=cambiaGirar(pidR,velL,velR)
        time.sleep(pidDeg.sample_time)
        #timeout
        if (time.time()-td0>20):
            print('cortar   !!!  Algo feo occurio en rumbando. Parandome')
            goMow=False
        if (time.time() - tJog > 5): #cannot use simple one-line
            jogNow = True
        
    print('cortar   Rumbo inicial esta completo')
    
    #Sumer velocidad
    # ~ #ramp up wheel speed to 75% max or until intended position is reached
    # ~ lejoStart=4*radiusMatch
    # ~ spdPerc=0
    # ~ while goMow and spdPerc<=0.75 and lejoStart>2*radiusMatch:#m
        # ~ spdPerc+=0.1
        # ~ lejoStart=gc.numLejo(gnaObj.get_position(),datPointNext)
        # ~ velL,velR=motoObj.vel(spdPerc*velL,spdPerc*velR)
        # ~ time.sleep(0.1)
    #kick speed up if started too close to first point
    if velL<=0.25*480:
        print(f'cortar  Necesite amplicar los velocidades un poco desde {velL}, {velR}')
        velL=0.75*480
        velR=-velL
        velL,velR=motoObj.vel(velL,velR)
    
    
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
        
        print(f'cortar  Yendo a {datPoint}')
        
        while goMow and not targetReached:
            
            #GNSS positioning
            gpsPointCurr=gnaObj.get_position()
            gpsPointChange=gc.numLejo(gpsPointPrev,gpsPointCurr) #m
            #continue after a reasonable change is detected
            if gpsPointChange>0.025:
                #reset
                gpsPointPrev=gpsPointCurr #lat, long
                #continue
                distToNextPoint=gc.numLejo(gpsPointCurr,datPoint) #m
                degHeadingIntend=gc.numRumbo(gpsPointCurr,datPoint) #brujula degrees
                print(f'cortar  Estoy a{round(distToNextPoint,2)}m a distancia en {gpsPointCurr} lat./lon. con intento a {round(degHeadIntend,0)} deg.')
                
                #Continue if point is in front of mower
                if degHeadingIntend<110:
                    timeSpdCurr=time.time() #for later
                    
                    #GNSS headings
                    #get gnss given heading
                    degHeadingGNSS=gnaObj.heading() #brujula degrees
                    headListIMU, headIndexIMU, degHeadGNSSAvg = numRollingAvg(headListIMU,headIndexIMU,degHeadingGNSS)
                    #determine actual heading
                    degHeadingActual=gc.numRumbo(gpsPointPrev,gpsPointCurr) #deg
                    headListPoints,headIndexPoints,degHeadActualAvg=numRollingAvg(headListPoints,headIndexPoints,degHeadingActual)
                    #give shortest spin angle to heading
                    if abs(numGirar(degHeadActualAvg,degHeadGNSSAvg))>30 and degHeadGNSSAvg!=0:
                        # ~ ledError.blink(on_time=onTime,off_time=flashMedium,n=10)
                        print('cortar    !!! GNSS que necesita ajuste, o giramos rapidamente.')
                        print('cortar         Rumbo observado: '+str(round(degHeadGNSSAvg)))
                        print('cortar         Rumbo calculado: '+str(round(degHeadActualAvg)))
                        degChangeReq=gc.numGirar(degHeadActualAvg,degHeadingIntend) #deg
                    else:
                        degChangeReq=gc.numGirar(degHeadingGNSS,degHeadingIntend) #deg
                                
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
                    print(f'cortar  Estoy progresando con ({velL}, {velR}) m/s a {round(degHeadActualAvg,0)} deg.')
                    
                    #reset
                    errPerd=0
                    errMovi=0
                    targetReached=(distToNextPoint<=radiusMatch)
                
                else: #regard the point as a miss
                    errPerd+=1
                    targetReached=True
            
            else:
                time.sleep(rato/2)
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
    
    print('cortar   fin')
    #Cortar completa
    return goMow


#late build globals
motThread=threading.Thread(target=cambiaControl, args=([60]))
btThread=threading.Thread(target=btApp.main,args=())
wtThread=threading.Thread(target=enviarTicks,args=())


if __name__ == '__main__':
    print('__PROBANDO__ mower')
    
    print(datetime.date.today())
    #print(datetime.datetime.now())
    motoObj=motores.motoresObj(True)
    
    try:
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
        # ~ motoObj.prueba=False
        # ~ btThread.start()
        # ~ cambiaControl(10,True)
        '''
        #probando motores
        # ~ print('motos incendido')
        # ~ for y in range(50,360,10):
            # ~ print(y)
            # ~ velL, velR=motoObj.vel(y,-y)
            # ~ time.sleep(0.02)
        # ~ motoObj.motPara()
        # ~ print('motos apagado')
        
        #probando ubicacion
        # ~ gnaObj, sendQ = gnss2.gnssIniciar(gnssPara)
        # ~ yp0=None
        # ~ yf0=None
        # ~ yh0=None
        # ~ for y in range(0,10):
            # ~ vp=gnaObj.get_position()
            # ~ vf=gnaObj.fixType
            # ~ vh=gnaObj.heading
            # ~ print(f'pos {vp}, fix {vf}, hed {vh}')
            # ~ yp1=type(vp)
            # ~ if yp0!=yp1:
                # ~ print(yp1)
                # ~ yp0=yp1
            # ~ yf1=type(vf)
            # ~ if yf0!=yf1:
                # ~ print(yf1)
                # ~ yf0=yf1
            # ~ yh1=type(vh)
            # ~ if yh0!=yh1:
                # ~ print(yh1)
                # ~ yh0=yh1
            # ~ time.sleep(1)
        # ~ gnssPara.set()
        # ~ time.sleep(2)
        # ~ gnaObj.stop()
        
        #probando conexiones
        # ~ print(f'btnCortar {btnCortar.is_pressed}')
        # ~ print(f'btnRecordar {btnRecordar.is_pressed}')
        # ~ histLeft=[0.001 for i in range(12)]
        # ~ histRight=[0.002 for i in range(12)]
        # ~ histIndexL=0
        # ~ histIndexR=0
        # ~ for i in histLeft:
            # ~ cmdL=round(potLeft.value,3)
            # ~ cmdR=round(potRight.value,3)
            # ~ print(cmdL, cmdR)
            # ~ histLeft,histIndexL,avgL=numRollingAvg(histLeft,histIndexL,cmdL)
            # ~ histRight,histIndexR,avgR=numRollingAvg(histRight,histIndexR,cmdR)
            # ~ time.sleep(0.01)
        # ~ print(f'potLeft {avgL}')
        # ~ print(f'potRight {avgR}')
        # ~ ledBeacon.on()
        # ~ print(f'ledBeacon {ledBeacon.value}')
        # ~ time.sleep(1)
        # ~ ledBeacon.off()
        # ~ ledBeacon.close()
        # ~ print(sc.run(["pinctrl set 19 pd"],shell=True,text=True,check=True))
        '''
        #probando cortar
        print(mowerInic())
    
    finally:
        motoObj.paraSuave()
        print('__prueba completa__')
else:
    fecha=datetime.datetime.now()
    motoObj=motores.motoresObj()
    print('mower    Acabo de empezar')
    
    import sys
    stdoutOld=sys.stdout
    fechaLog=fileMan.prefixMatch(arcRuta+'logs/',str(fecha))
    logFile=fechaLog+'.txt'
    mowerLog=open(logFile,'w')
    sys.stdout=mowerLog
    logThread=threading.Thread(target=logSave,args=([mowerLog, motoPara]))
    logThread.start()
    
    print('__AUTONOMY__ mower')
    print(fecha)
    hora0=time.time()
    sinErrores=mowerInic()
    mowerPara(not sinErrores)
    horaTotal=time.time()-hora0
    print(f'Horas en mower = {datetime.timedelta(seconds=horaTotal)}')
    print('__acabo__ mower')
    sys.stdout=stdoutOld
    mowerLog.close()
    #power down 
    sc.run(['shutdown -P'], shell=True, text=True, check=True)
