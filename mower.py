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
# ~ import btApp #end of support for pyBluez
#standardb
import time
import datetime
import gpiozero as gp
import os
import statistics
import threading
import subprocess as sc
import sys
#semi-custom
from simple_pid import PID
from pyubx2 import ubxtypes_core as ubt


#globals
arcRuta=os.path.dirname(os.path.abspath(__file__))+'/'
archivoRecupe=arcRuta + 'rutaPunto.txt'
radiusMatch=11*0.0254 #m, tamano de plataforma
rato=0.1 #sec, 10Hz iaRTN limit
velStd=0.8*0.5 #m/s
cmdPower=0 #/480
cmdSteer=0 #/480
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
    global gnaObj
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
    print(f'ctrlLuces   Rato por primera 3D fix: {round(horaTTF-horaEntrada,2)}s')
    #si necesita un fix de mas alta calidad
    if rtk_Req and sinErr:
        ledBeacon.blink(on_time=onTime,off_time=offMedium)
        print('ctrlLuces    Esperando por un 3D Fix de alta calidad')
        while ((not gnssPara.is_set()) and fT>=3): #should be fT!=4
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
        print(f'ctrlLuces  Rato por alta 3D fix: {round(time.time()-horaTTF,2)}s')
    #Avisar por encender el motor
    ratoAviso=10#s
    horaInic1=time.time()+ratoAviso
    nB=int((onTime+offShort)/ratoAviso)
    ledBeacon.blink(on_time=onTime,off_time=offShort,n=nB)
    print(f'ctrlLuces    Waiting {round(ratoAviso,1)}s before starting motors ...')
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

def cambiaGirar(pwr, steer, incr=False):
    '''
    input: pwr = from -100 (reverse) to 100 (forward)
        steer = -200 (CCW spin) to 200 (CW spin)
        incr = bool, change by given values instead of setting them to those values
    changes wheel velocities
    echoes commanded forward power level and left/right steering
    '''
    global motoObj
    
    if incr:
        return motoObj.vel(motoObj.fwdRev + pwr, motoObj.steer + steer)
    
    return motoObj.vel(pwr,steer)

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
    # passes rheostat (FORWARD ONLY) or Bluetooth inputs to motor control.
    # intended as an independent thread
    #returns nothing
    
    #inic
    print('cambiaControl    Iniciando cambiaControl con ratoSus='+str(ratoSus))
    timeCambia=time.time()
    ratoFin=False
    shutRequest=False
    shutStart=False
    global cmdPower, cmdSteer
    #tuned
    potLeftBase=0.145
    potLeftArriba=0.387
    potRightBase=0.128
    potRightArriba=0.311
    #tuners
    mm=[0,0]
    mn=[100,100]
    avl=[]
    avr=[]
    #suave
    histLeft=[potLeftBase]*6
    histRight=[potRightBase]*6
    histIndexL=0
    histIndexR=0
    adcLeft=0
    adcRight=0
    avgLeft=1
    avgRight=1
    
    while (not motoPara.is_set()) and (not ratoFin):
        if prueba:
            print('cambiaControl    __')
        #get remote values
        if False: #btApp.btReady: #remote control
            cmdPower=btApp.powerVal
            cmdSteer=btApp.steerVal
        #read and average pot switches
        else: #manual sticks (forward only)
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
            stickL=(avgLeft-potLeftBase)/(potLeftArriba-potLeftBase)*100
            stickR=-(avgRight-potRightBase)/(potRightArriba-potRightBase)*100
            cmdPower=(stickL-stickR)/1.0 #allows spinning on stationary wheel with full power
            cmdSteer=(stickL+stickR)*1.5
        
        #send inputs to controller 
        cmdPower,cmdSteer=motoObj.vel(cmdPower,cmdSteer)
        if prueba:
            # ~ print(f"cambiaControl    btApp.btReady = {btApp.btReady}")
            # ~ print(f"cambiaControl    btApp.powerVal = {btApp.powerVal}")
            # ~ print(f"cambiaControl    btApp.steerVal = {btApp.steerVal}")
            print('cambiaControl    adcs='+str([round(adcLeft,2),round(adcRight,2)]))
            print('cambiaControl    avgs='+str([round(avgLeft,2),round(avgRight,2)]))
            print('cambiaControl    vels='+str([motoObj.vel1, motoObj.vel2]))
                
        #throw timeout flag
        timeElapsed=time.time()-timeCambia
        if prueba:
            # ~ shutStart=True
            print('cambiaControl    ET='+str(round(timeElapsed,2)))
            print(f"cambiaControl    ratoSus = {ratoSus}")
        #seek requests for shutdown
        # ~ if (btApp.btReady and abs(btApp.powerVal)<=5): #priority to remote control
            # ~ shutRequest=True
        if ((abs(avgLeft)<1.015*potLeftBase) and (abs(avgRight)<1.020*potRightBase)):
            shutRequest=True
        else:
            shutRequest=False
        #act on said requests
        if shutRequest:
            if (not shutStart) and (ratoSus>0):
                print('cambiaControl     Throttle shutdown bandera sacado en '+str(round(timeElapsed,1)))
                timeCambia=time.time()
                shutStart=True
        else:
            if shutStart and ratoSus>0:
                print('cambiaControl     Throttle shutdown bandera despejado en '+str(round(timeElapsed,1)))
                shutStart=False
        #late test check
        if prueba:
            print(f"cambiaControl    shutStart = {shutStart}")
            
        ratoFin=((ratoSus>0) and (time.time()-timeCambia>ratoSus) and shutStart)
        
        #don't overload anything
        time.sleep(0.1*rato)
        
    #report exits
    dt=''
    if motoPara.is_set():
        dt+=' motoPara'
        autonomy=True
    else:
        autonomy=False
    if ratoFin:
        dt+=' ratoFin'
        motoObj.paraSuave(leaveOn=autonomy)
    print(f'cambiaControl    Saliendo cambiaControl por{dt}')
    
    if prueba:
        print(f'cambiaControl          L     R')
        print(f'cambiaControl    min: {round(mn[0],3):04.3f}, {round(mn[1],3):04.3f}')
        print(f'cambiaControl    max: {round(mm[0],3):04.3f}, {round(mm[1],3):04.3f}')
        mfl=statistics.fmean(avl)
        mfr=statistics.fmean(avr)
        print(f'cambiaControl    avg: {round(mfl,3):04.3f}, {round(mfr,3):04.3f}')
        print(f'cambiaControl    alp: {round((mfl-potLeftBase)/(potLeftArriba-potLeftBase),2):.0%}, {round((mfr-potRightBase)/(potRightArriba-potRightBase),2):.0%}')
    
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
    
    def _enviar(cmdPower,cmdSteer):
        #normal forward = positive
        wtL=int(abs(round(cmdPower*tickMult,0)))
        wdL=(cmdPower>=0)
        #normal forward = negative
        wtR=int(abs(round(cmdSteer*tickMult,0)))
        wdR=(cmdSteer<0)
        
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
    
def mowerInic(prueba=False):
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
    # ~ btThread.start()
    #warm up GNSS
    gnaObj = gnss.gnssIniciar(gnssPara)#,verbose=prueba)
    # ~ wtThread.start()
    
    #Choose the Mode
    #follow toggle orders
    horaVia=time.time()
    horaEn=0
    horaLim=240#s
    archivoGuia=arcRuta+'rutaSeguida.txt'
    opcion=False
    
    print('mowerInic    Esperando por eligiones humanos...')
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
                sinErrores, archivoNuevo=recordar(verbose=prueba)
                recArchivo=open(archivoGuia,'w')
                recArchivo.write(archivoNuevo)
                recArchivo.close()
            else:
                sinErrores=False
            
        elif btnCortar.is_pressed: #cut
            opcion=True
            print('mowerInic    btnCortar eligido')
            motoPara.set()
            # ~ btApp.cerrar()
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
                    for idx, linea in enumerate(arcSrc):
                        d=linea[:-1]
                        l=d.split()
                        ll=[float(x) for x in l]
                        datPointList.append(ll)
                #check for unfinished business
                with open(archivoRecupe,'r') as rP:
                    puntoUltimo=rP.readline() #last recorded index, enables recovery
                    if (puntoUltimo>=int(0.95*len(datPointList)) or puntoUltimo is None):
                        puntoUltimo=0
                #cut
                print(f'mowerInic   siguiendo {len(datPointList)} puntos en linea')
                sinErrores=cortar(datPointList,puntoUltimo,prueba)
            else:
                print('mowerInic    !!! No pude continuar con cortar sin GPS')
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

def mowerPara(errStop,autonomous=False):
    #release of manual controls and possible motion shutdown
    
    print(f'mowerPara   Detente solicitado con emergencia = {errStop}')
    
    #cut engines
    if errStop:
        motoObj.motPara()
    elif not autonomous:
        motoObj.paraSuave()
    print('mowerPara    Motores han sido apagados')
    if motoObj.errores:
        errStop=True
    
    #quit controls
    motoPara.set()
    print('mowerPara    Controles manuales apagando')
    # ~ if btApp.btReady:
        # ~ btApp.cerrar()
    # ~ print('mowerPara    BT conexion ha sido apagado')
    
    #late shutdowns
    if not autonomous:
        global cmdPower, cmdSteer
        cmdPower=0
        cmdSteer=0
        
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

def recordar(prlist=[],verbose=False):
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
        points=0
        while btnRecordar.is_pressed==True:
            time.sleep(rato)
            #dd=gnss.getPosition()
            dd=gnaObj.get_position()
            #skip very close recordings
            larga=gc.numLejo(dd,ddV)#m
            if larga>=0.05:
                datu=' '.join(str(ll) for ll in dd)
                if verbose:
                    dh = round(gnaObj.heading,0)
                    datu = datu + ' ' + str(dh)
                    print(f'recordar    Punto {dd} en rumbo {dh}')
                a.write(datu+'\n')
                ddV=dd
                points+=1
            elif verbose:
                print(f'recordar    Punto {dd} no es suficientamente lejo del previo')
            if points>=24: #periodically save
                a.flush()
                points=0
    
    print('recordar recordar completo')
    
    #complete
    motoPara.set() #backup
    if motThread.is_alive():
        motThread.join()
    # ~ ledProcess.on()
    
    print('recordar fin')
    
    return hazRec, archivoRuta

def cortar(datPointList, datPointStart=0, verbose=False):
    #follows given coordinate list
    #returns general status
    
    #Scope init
    goMow=True
    datPointList=datPointList[datPointStart:]
    #ruedas
    global cmdPower, cmdSteer
    #cmdPower=0 #pos. forward
    #cmdSteer=0 #neg. left
    pidDeg=PID(0.875,0.001,0.21,setpoint=0)
    pidDeg.sample_time=rato
    pidDeg.output_limits=(-200,200) #180deg = 3.0sec @ 7.9 RPM
    
    #GNSS locked
    # ~ ledGNSS.blink(onTime, offLong)
    ledBeacon.blink(on_time=onTime, off_time=offMedium)
    
    #First spin to first point
    headDel=180
    #get current position
    gpsPointPrev=gnaObj.get_position()
    #get intended positions
    datPointNext=datPointList[datPointStart] #lat, long
    td0=time.time()
    #init backup heading finders
    tJog=td0
    jogNow=True
    gpsHead=0
    #spin to win
    print(f'cortar   Iniciando desde {gpsPointPrev} lat/lon')
    print(f'cortar   Primer punto es {datPointNext} lat/lon')
    while goMow and (abs(headDel)>5):
        #determine required heading to intended position
        rumPrimera=gc.numRumbo(gpsPointPrev,datPointNext) #deg
        # ~ gpsHead=gnss.getHeading() #deg
        if gnaObj.heading==0 or jogNow: #bad sensor reading / first move
            if jogNow:
                #jog the wheels
                print('cortar   [A PRIMER PUNTO] Voy a intentar un jog')
                cmdPower, cmdSteer=motoObj.vel(50,0)
                time.sleep(2)
                cmdPower, cmdSteer=motoObj.vel(25,0,True)
                #eval
                gpsPointJog=gnaObj.get_position()
                if gc.numLejo(gpsPointPrev,gpsPointJog)<0.05:
                    print(f'cortar  [A PRIMER PUNTO] cero desde cero, ({round(gpsPointJog[0],0)}, {round(gpsPointJog[1],0)}) lat/lon')
                else:
                    print(f'cortar  [A PRIMER PUNTO] de {gpsPointPrev} hasta {gpsPointJog} lat/lon')
                    gpsHead=gc.numRumbo(gpsPointPrev,gpsPointJog)
                    print(f'cortar  [A PRIMER PUNTO] durante aproximamente {round(gpsHead,0)} deg')
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
        if verbose:
            print(f'cortar  [A PRIMER PUNTO] pidR={round(pidR,3)}: Tenemos {round(headDel,0)} deg. por llegar a {round(rumPrimera,0)} deg. desde {round(gpsHead,0)} deg. con cmd {int(cmdPower)}, {int(cmdSteer)}')
        cmdPower,cmdSteer=cambiaGirar(cmdPower,pidR)
        time.sleep(pidDeg.sample_time)
        #timeout
        if (time.time()-td0>20):
            print('cortar   [A PRIMER PUNTO] !!!  Algo feo occurio en 20 sec. de rumbando. Parandome')
            goMow=False
        if (time.time() - tJog > 5): #cannot use simple one-line
            jogNow = True
        
    print('cortar   [A PRIMER PUNTO] Rumbo inicial esta completo')
    
    #Sumer velocidad
    if abs(cmdPower)<=25 and goMow:
        print(f'cortar  [TRANSICION] Necesite amplicar el poder un poco desde {cmdPower}, {cmdSteer}')
        cmdPower,cmdSteer=motoObj.vel(75,0)
    else:
        cmdPower,cmdSteer=motoObj.vel(cmdPower,0)
        print(f'cortar  [TRANSICION] El commands ya era {cmdPower}, {cmdSteer}. goMow = {goMow}')
        # ~ cmdPower,cmdSteer=motoObj.vel(0,0)
    
    
    #Iniciar cortar
    targetReached=False
    errPerd=0
    errMovi=0
    #speed mgmt
    velCurr=velStd
    velAvg=[velCurr for i in range(40)]
    velAvgIndex=0
    timeSpdPrev=time.time()
    #heading mgmt
    degHeadActualAvg=gpsHead
    headListIMU=[gpsHead+1 for i in range(8)]
    headIndexIMU=0
    '''headListPoints=[gpsHead-1 for i in range(12)]
    headIndexPoints=0
    #position mgmt (from A to B)
    pointCloudBX=[gpsPointPrev[0] for i in range(15)]
    pointCloudBY=[gpsPointPrev[1] for i in range(15)]
    pointCloudIndexB=0
    pointCloudBmeanX=0
    pointCloudBmeanY=0
    #signal and start
    # ~ ledProcess.blink(onTime,offLong)
    '''
    klen=len(datPointList)
    print(f'cortar  [PRIMER a ATRAVESAR] Empezando con una lista de largo {klen}')
    for k, datPoint in enumerate(datPointList):
        
        if goMow:
            print(f'cortar  [ATRAVESAR] Yendo a ({k:{" "}>3}/{klen}) {datPoint} lat/lon con ({cmdPower}, {cmdSteer}) fwd/turn')
        
        while goMow and not targetReached:
            
            #GNSS positioning
            gpsPointCurr=gnaObj.get_position()
            '''pcib = pointCloudIndexB
            pointCloudBX, pcib, pointCloudBmeanX = numRollingAvg(pointCloudBX, pointCloudIndexB, gpsPointCurrDot[0])
            pointCloudBY, pcib, pointCloudBmeanY = numRollingAvg(pointCloudBY, pointCloudIndexB, gpsPointCurrDot[1])
            pointCloudIndexB = pcib
            gpsPointCurr=[round(pointCloudBmeanX,7), round(pointCloudBmeanY,7)]
            '''
            gpsPointChange=gc.numLejo(gpsPointPrev,gpsPointCurr) #m
            #continue after a reasonable change is detected
            if verbose:
                print(f'cortar  [ATRAVESAR] Estoy en {gpsPointCurr} lat/lon, y movi {round(gpsPointChange,2)}m de distancia con ({int(cmdPower)}, {int(cmdSteer)} fwd/turn')
            if gpsPointChange>0.11: #~0.02m / tan(10deg)
                #reset
                gpsPointPrev=gpsPointCurr #lat, long
                #continue
                distToNextPoint=gc.numLejo(gpsPointCurr,datPoint) #m
                degHeadingIntend=gc.numRumbo(gpsPointCurr,datPoint) #brujula degrees
                if verbose:
                    print(f'cortar  [ATRAVESAR] Estoy a {round(distToNextPoint,2)}m a distancia y en {gpsPointCurr} lat./lon. con intento a {round(degHeadingIntend,0)} deg.')
                
                #Continue if point is in front of mower
                if abs(gc.numGirar(degHeadActualAvg,degHeadingIntend))<150:
                    timeSpdCurr=time.time() #for later
                    
                    #GNSS headings
                    #get gnss given heading
                    degHeadingGNSS=gnaObj.heading #brujula degrees
                    headListIMU, headIndexIMU, degHeadGNSSAvg = numRollingAvg(headListIMU,headIndexIMU,degHeadingGNSS)
                    '''#determine actual heading
                    degHeadingActual=gc.numRumbo(gpsPointPrev,gpsPointCurr) #deg
                    headListPoints,headIndexPoints,degHeadActualAvg = numRollingAvg(headListPoints,headIndexPoints,degHeadingActual)
                    #give shortest spin angle to heading
                    if abs(gc.numGirar(degHeadActualAvg,degHeadGNSSAvg))>30 and degHeadGNSSAvg!=0:
                        # ~ ledError.blink(on_time=onTime,off_time=flashMedium,n=10)
                        print('cortar    [ATRAVESAR] !!! GNSS que necesita ajuste, o giramos rapidamente.')
                        print('cortar    [ATRAVESAR]      Rumbo observado: '+str(round(degHeadGNSSAvg)))
                        print('cortar    [ATRAVESAR]      Rumbo calculado: '+str(round(degHeadActualAvg)))
                        degChangeReq=gc.numGirar(degHeadActualAvg,degHeadingIntend) #deg
                    else:
                        degChangeReq=gc.numGirar(degHeadingGNSS,degHeadingIntend) #deg
                    '''
                    degChangeReq=gc.numGirar(degHeadingGNSSAvg,degHeadingIntend)
                                
                    #Twist the wheels
                    # input of heading differential (degree value amount needed)
                    pidR=pidDeg(degChangeReq)
                    cmdPower,cmdSteer=cambiaGirar(cmdPower,pidR)
                    # check centerline speed to standard
                    velCurr = gnaObj.speed/1000 #mm/s --> m/s
                    if gnaObj.speed==0: #backup in case of lost reading
                        velCurr=gc.numLejo(gpsPointPrev,gpsPointCurr)/(timeSpdCurr-timeSpdPrev)#m/s
                    timeSpdPrev=time.time()
                    velAvg,velAvgIndex,velAvgAvg=numRollingAvg(velAvg,velAvgIndex,velCurr)
                    if velAvgAvg<velStd:
                        cmdPower,cmdSteer=motoObj.vel(0.1*cmdPower,0,True)
                    elif velAvgAvg>=1.2*velStd:
                        cmdPower,cmdSteer=motoObj.vel(-0.1*cmdPower,0,True)
                    #finally
                    if verbose:
                        print(f'cortar  [ATRAVESAR] Estoy progresando con {round(velAvgAvg,2)} m/s ({cmdPower}, {cmdSteer}) m/s a {round(degHeadActualAvg,0)} deg. pidiendo pidR = {pidR}')
                    
                    #reset
                    errPerd=0
                    errMovi=0
                    targetReached=(distToNextPoint<=radiusMatch)
                
                else: #regard the point as a miss
                    if verbose:
                        print('cortar   [ATRAVESAR] Queda detras de me. Olvidandolo')
                    errPerd+=1 #I accept that a string of lost points on a very sharp turn is possible
                    targetReached=True
                    
                if verbose and targetReached:
                    print(f'cortar  [FIN] Hemos llegado a {datPoint}')
                    targetReached=False
            
            else:
                # ~ print(f'cortar   [ATRAVESAR] No era suficiente. Commands {cmdPower}, {cmdSteer} fwd/turn')
                # ~ cmdPower=int(cmdPower*1.002) #bump speed
                # ~ cmdPower,cmdSteer=motoObj.vel(cmdPower,cmdSteer)
                #give it a second
                time.sleep(rato/2)
                errMovi+=1
                if errMovi>4/(rato/10):
                    print('cortar   [FIN] !!! Estoy atascado. Auxilio, porfa.')
                    goMow=False
            #end GNSS positioning
            
            if errPerd>20:
                # ~ ledError.on()
                print('cortar   [FIN] !!! Muchos puntos perdidos en sequencia. Parandome')
                goMow=False
            
            if not btnCortar.is_pressed:
                # ~ ledProcess.blink(on_time=onTime,off_time=offShort)
                print('cortar   [FIN] !!! Conexion a btnCortar perdido. Parandome')
                goMow=False
            
            #continua a punto
            
        #record last point reached
        with open(archivoRecupe,'w') as ar:
            ar.write(str(k+datPointStart))
        
        # ~ print('cortar   [FIN] ____________')
        
    #sigue con proximo punto
    
    print('cortar   fin')
    #Cortar completa
    return goMow

def testHeading(headings, timePer):
    #moves in direction of given headings (compass bearings for specified amount of time (seconds)
    #returns ((lat, lon), heading, speed)
    
    def catResults(latlon1, head1, headInt, spd1):
        gps = (round(latlon1[0],7), round(latlon1[1],7))
        head = round(head1,0)
        speed = round(spd1/1000,3)
        return [gps, head, headInt, speed]
    
    # Initialize
    gnaObj = gnss.gnssIniciar(gnssPara)
    global cmdPower, cmdSteer
    jogNow = False
    testSpeed = 100 #/100
    results=[]
    dwell = 0.1 #sec.
    
    # Velocity control
    pidDeg=PID(0.875,0.001,0.21,setpoint=0) #0.75,0.001,0.1 | .875,.001,.21
    pidDeg.sample_time=dwell
    stepmax=200
    pidDeg.output_limits=(-stepmax,stepmax) #unity
    
    # Heading smoothing
    headListIMU=[headings[0]+1 for i in range(8)]
    headIndexIMU=0
    gpsHeadAvg = 0
    
    # Start wheels
    cmdPower, cmdSteer = cambiaGirar(testSpeed, 0)
    time.sleep(1)
    
    # Move through bearings
    gpsPointPrev = gnaObj.get_position()
    h = len(headings) - 1
    for i, compass in enumerate(headings):
        
        td0 = time.time()
        print(f'testHeading {compass} for {round(timePer,1)} sec.' )
        
        while time.time() <= td0 + timePer:
            
            # Wait a moment
            td1 = time.time()
            time.sleep(dwell)
            
            # Take a snapshot
            gpsPoint = gnaObj.get_position()
            gpsHead = gnaObj.heading
            gpsSpeed = gnaObj.speed
            td2 = time.time()
            
            # In case of no heading
            while gpsHead==0 or jogNow: 
                #jog the wheels
                print('testHeading   [JOGGING] Bad heading sensor reading. Going to move straight.')
                cmdPower, cmdSteer=motoObj.vel(testSpeed,0)
                time.sleep(2)
                
                #eval
                gpsPointJog=gnaObj.get_position()
                if gc.numLejo(gpsPoint,gpsPointJog)<0.05: #it did not move enough
                    print(f'testHeading  [JOGGING] No movement at ({round(gpsPointJog[0],0)}, {round(gpsPointJog[1],0)}) lat/lon. Trying again.')
                    jogNow = True
                else: #manually calculate gps things
                    gpsHead=gc.numRumbo(gpsPoint,gpsPointJog)
                    gpsSpeed=gc.numLejo(gpsPointPrev,gpsPointJog)/(time.time()-td2)/1000
                    print(f'testHeading  [JOGGING] from {gpsPoint} to {gpsPointJog} lat/lon gave {round(gpsHead,0)} deg. at {int(gpsSpeed)} mm/s')
                    gpsPoint = gpsPointJog
                    td2 = time.time()
                    jogNow = False
                    
            # Move on with normal corrections    
            #smooth out heading readings
            headListIMU, headIndexIMU, gpsHeadAvg = numRollingAvg(headListIMU,headIndexIMU,gpsHead)
            #new wheel movement
            headDelta=gc.numGirar(gpsHeadAvg,compass) #deg
            pidR=int(pidDeg(headDelta/1)*1) #scale unity out of 480
            cmdPower,cmdSteer=cambiaGirar(testSpeed,pidR)
            
            # Save results
            datSet = catResults(gpsPoint, gpsHeadAvg, compass, gpsSpeed)
            results.append(datSet)
            gpsPointPrev = gpsPoint
            print(f'testHeading   ({i}/{h}) pidR={pidR:>4} makes cmd ({cmdPower:>4}, {cmdSteer:>4}) when delta={int(headDelta):>4} deg. (to {int(compass):>3} deg. from {int(gpsHeadAvg):>3} deg.) at {gpsSpeed:>4} mm/s')
        
        #fake save for recovery
        with open(archivoRecupe,'w') as ar:
            ar.write(str(i))
            
    cmdPower, cmdSteer = cambiaGirar(0,0)
    print('testHeading   Completed')
    
    return results

#late build globals
motThread=threading.Thread(name='motThread',target=cambiaControl, args=([60]))
# ~ btThread=threading.Thread(name='btThread',target=btApp.main,args=())
# ~ btThread.daemon=True
# ~ wtThread=threading.Thread(name='wtThread',target=enviarTicks,args=())


if __name__ == '__main__':
    print('__PROBANDO__ mower')
    
    print(datetime.date.today())
    #print(datetime.datetime.now())
    motoObj=motores.motoresObj(True)
    
    '''try:
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
        
        #probando motores
        # ~ print('motos incendido')
        # ~ for y in range(50,360,10):
            # ~ print(y)
            # ~ cmdPower, cmdSteer=motoObj.vel(y,-y)
            # ~ time.sleep(0.02)
        # ~ motoObj.motPara()
        # ~ print('motos apagado')
        
        #probando ubicacion
        # ~ print('TEST location')
        # ~ gnaObj = gnss.gnssIniciar(gnssPara)
        # ~ yp0=None
        # ~ yf0=None
        # ~ yh0=None
        # ~ for y in range(0,10):
            # ~ vp=gnaObj.get_position()
            # ~ vf=gnaObj.fixType
            # ~ vh=gnaObj.heading
            # ~ print(f'pos {vp}, fix {vf}, head {vh}')
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
        # ~ print('TEST location complete')
        
        #probando conexiones
        print(f'btnCortar {btnCortar.is_pressed}')
        print(f'btnRecordar {btnRecordar.is_pressed}')
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
    
    #testing follow a bearing
    # ~ resHead = testHeading([0,270,180,90],5)
    # ~ from plotCoordMap import plot_vectors
    # ~ plot_vectors(resHead)
    
    # ~ #probando cortar
    print(mowerInic(True))
    
    # ~ finally:
    print('__casi completa__')
    mowerPara(False)
    print('__prueba completa__')
    for t in threading.enumerate():
        print(f'Active thread: {t.name}')
    sys.exit()
else:
    fecha=datetime.datetime.now()
    motoObj=motores.motoresObj()
    print('mower    Acabo de empezar')
    
    stdoutOld=sys.stdout
    fechaLog=fileMan.prefixMatch(arcRuta+'logs/',str(fecha))
    logFile=fechaLog+'.txt'
    mowerLog=open(logFile,'w')
    sys.stdout=mowerLog
    logThread=threading.Thread(name='logThread',target=logSave,args=([mowerLog, motoPara]))
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
