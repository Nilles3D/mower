#!/home/rpi32/Documents/env/bin/python
# -*- coding: utf-8 -*-
#
#  motores.py
#  
#  Copyright 2024  <rpi31@rpi31>
#  
#  motor command parsing and execution
#  

from dual_tb9051ftg_rpi import motors
from simple_pid import PID
from time import sleep, time
from threading import Thread, Event

geared = -1 #hace la opuesta girar

class motoresObj:
    
    def __init__(self, probar=False):
        print('motores  Unos motores han sido iniciados')
        
        self.prueba=probar
        
        self.errores=False
        
        #velocidades iniciadas
        self.vel1=0 #ordenado
        self.vel1act=0 #actual
        self.vel2=0
        self.vel2act=0
        self.velMax=motors.MAX_SPEED
        
        #controls
        self._rato=0.01#s
        self._monitorPara=Event()
        self._monT=Thread(target=self._velMonitor,args=())
        
        #iniciar
        self._timeT=Thread(target=self._timer,args=())
        if probar:
            self._timeT.start()
        self._motInic()
        
        return
    
    def __enter__(self):
        return
    
    def __exit__(self):
        self.paraSuave()
        return
    
    def _motInic(self):
        print('motores  Habilitando motores')
        motors.enable()
        if motors.getFaults():
            self.errores=True
            if motors.motor1.getFault():
                motNum=1
                motN2=' y 2' if motors.motor2.getFault() else ''
            else:
                motNum=2
                motN2=''
            print(f'motores  !!!Error detectado en motor {motNum}{motN2}. Parando motores')
            self.motPara()
        else:
            self._monitorPara.clear()
            self._monT.start()
        return
    
    def _velPon(self,vel_1,vel_2):
        self.vel1act=max(-self.velMax,min(int(vel_1),self.velMax))
        self.vel2act=max(-self.velMax,min(int(vel_2),self.velMax))
        motors.setSpeeds(geared*self.vel2act,geared*self.vel1act) #connected backwards IRL
        return
    
    def _velMonitor(self):
        print('motores  Monitor empezando')
        mf1=0
        mf2=0
        #iniciar
        kMot=[0.05,0,0]
            #[0.33,0.66,0.0001] with step of 5 is very smooth
            #[1,0.5,0.0001] with step of 20 feels just fast enough
            #[0.05,0,0] with step 15 is smooth for digital commands
        maxCambia=15 # cambia de velocidad permisible en rato
            
        pidMot1=PID(kMot[0],kMot[1],kMot[2],setpoint=0)
        pidMot2=PID(kMot[0],kMot[1],kMot[2],setpoint=0)
        pids=[pidMot1,pidMot2]
        for p in pids:
            p.output_limits=[-1,1] #unidad
            p.sample_time=self._rato
        
        #actualizar
        while not self._monitorPara.is_set():
            #buscar solucion
            pid1=pidMot1(self.vel1act-self.vel1)*maxCambia #%*velDel
            pid2=pidMot2(self.vel2act-self.vel2)*maxCambia
            # ~ if self.prueba:
                # ~ print(f'motores  pid1={pid1:.2f}, vel1act={self.vel1act:03.0f}, vel1={self.vel1:03.0f}')
            #cambiar los giradores
            cmd1=self.vel1act+pid1
            cmd2=self.vel2act+pid2
            self._velPon(cmd1,cmd2)
            #esperar
            sleep(self._rato)
            
            #check for faults
            if motors.motor1.getFault():
                mf1+=1
                print(f'motores  motor1 has had {mf1} faults')
                motors.disable()
                sleep(self._rato)
                motors.enable()
            if motors.motor2.getFault():
                mf2+=1
                print(f'motores  motor2 has had {mf2} faults')
                motors.disable()
                sleep(self._rato)
                motors.enable()
            if mf1>=100 or mf2>=100:
                print('motores  Excesso de errores. Parandome.')
                self.motPara()
            
        
        print('motores  Monitor en fin')
        return
    
    def _timer(self):
        t0=time()
        while not self._monitorPara.is_set():
            sleep(0.25)
            print(f'motores time = {time()-t0:.2f}')
        return
    
    def vel(self,vel_1,vel_2):
        self.vel1=vel_1
        self.vel2=vel_2
        
        if self.prueba:
            print(f'motores  dado {self.vel1}, {self.vel2}')
            sleep(10*self._rato)
            print(f'motores  usar {self.vel1act}, {self.vel2act}')
        
        return vel_1, vel_2
    
    def paraSuave(self):
        print('motores  Parando suavemente')
        self.vel(0,0)
        sleep(1.1)
        self._monitorPara.set()
        if self._monT.is_alive():
            self._monT.join()
        if self._timeT.is_alive():
            self._timeT.join()
        self.vel1act=0
        self.vel2act=0
        print(f'motores Faults antes de motPara(): {motors.getFaults()}')
        self.motPara()
        return
    
    def motPara(self):
        motors.setSpeeds(0,0)
        motors.disable()
        self._monitorPara.set()
        print('motores  Motores parados')
        return 0


if __name__ == '__main__':
    print('__ Motores ha empezado')
    import matplotlib.pyplot as plt
    
    try:
        ruedas=motoresObj(True)
        
        #visualize
        fig, ax = plt.subplots()
        
        #square waves
        ySq=[0]
        x1=[0]
        y1=[0]
        t1=time()
        for sqVel in [-480*.75, -480*.6, -480*.1, -480*1.0, 0]:
            ruedas.vel(sqVel,0)
            t1b=time()
            while (time()-t1b<1.2):
                x1.append(round(time()-t1,2))
                y1.append(ruedas.vel1act)
                ySq.append(sqVel)
                sleep(ruedas._rato)
        ax.plot(x1,ySq,linewidth=1.0)
        ax.plot(x1,y1,linewidth=2.0)
        
        #multi-step triangle wave
        ruedas.vel(0,0)
        sleep(1.5)
        yTr=[0]
        x2=[0]
        y2=[0]
        velRampa=list(range(0,480,5))#+list(range(480,0,-5))#+list(range(-480,0,5))+[0]
        t2=time()
        for v in velRampa:
            ruedas.vel(0,v)
            sleep(ruedas._rato/2)
            x2.append(round(time()-t2,2))
            y2.append(ruedas.vel2act)
            yTr.append(v)
        ax.plot(x2,yTr,linewidth=1.0)
        ax.plot(x2,y2,linewidth=2.0)
            
        print(f'faults antes de paraSuave(): {motors.getFaults()}')
        ruedas.paraSuave()
        print(f'despues de paraSuave(): {motors.getFaults()}')
        
        
        plt.show()
        
        ruedas.paraSuave()
    
    except KeyboardInterrupt:
        ruedas.motPara()
    
    finally:
        motors.forceStop()
    
    ('__ Motores ha parado')
