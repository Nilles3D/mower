#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  btApp.py
#  
#  Copyright 2024  <rpi31@rpi31>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#  

#credit
#https://community.appinventor.edu/t/raspberry-pi-bluetooth-send-receive/59846/4
#https://scribles.net/setting-up-blueooth-serial-port-profile-on-raspberry-pi/

import bluetooth as bt

#bt setup
btGo=True
btReady=False
server_sock=bt.BluetoothSocket(bt.RFCOMM)
port=22
server_sock.bind(("",port))
haConectado=False

#percent controls
#keys
powerCode=0
biasCode=1
steerCode=2
closeCode=9
#values
powerVal=0
steerVal=0
biasVal=0

def iniciar():
    global client_sock
    global btReady
    global haConectado
    
    if not haConectado:
        server_sock.listen(1)
    else:
        print("BtApp    Buscando otra conexion...")
    client_sock,address=server_sock.accept()
    print("btApp    Conexion realizada con: ",address)
    
    btReady=True
    haConectado=True
    
    return

def cerrar():
    global btReady
    global btGo
    print("btApp    cerrando...")
    btReady=False
    btGo=False
    client_sock.close()
    server_sock.close()
    print("btApp    cerrado")
    return
    
def recibir(prueba=False):
    global powerVal
    global steerVal
    global biasVal
    global btGo
    global btReady
    
    nuevo = False #retry the connection
    
    try:
        print(f"btApp    Empezando con power={powerVal}, steer={steerVal}, y bias={biasVal}")
        while len(client_sock.getpeername())>0 and btGo:
            # ~ print("wait")
            recvdata=client_sock.recv(256)
            #on receipt
            recCode=recvdata.decode()
            # ~ print(recCode)
            if len(recCode)>1:
                #code
                recVar=int(recCode[:1])
                #value sanitization
                recVal=recCode[1:]
                lastDash=recVal.rfind("-")
                #value conversion
                recNum=int(recVal[max(0,lastDash):])
            else:
                recVar=0
                recNum=0
            #use
            if prueba:
                print(recVar, recNum)
            if (type(recNum) is int):
                if ((-200<=recNum) and (recNum<=200)):
                    if recVar==powerCode:
                        powerVal=recNum
                    elif recVar==biasCode:
                        biasVal=recNum
                    elif recVar==steerCode:
                        steerVal=recNum+biasVal
                    else:
                        print(f"btApp    {recVar}, {recNum}")
            #exit attempt
            if recVar==closeCode:
                cerrar()
                return False
    except Exception as err:
        errPhrase=str(err)
        print("btApp    ",errPhrase)
        dropout="[Errno 104]"
        nuevo = (errPhrase[:len(dropout)]==dropout)
    except KeyboardInterrupt:
        print("btApp    saliendo")
    finally:
        btReady = False
        if not nuevo:
            cerrar()
    
    return nuevo


def main(prueba=False):
    print("btApp    main inicio")
    global btGo
    
    #first connection
    iniciar()
    
    intentar=True
    while intentar and btGo:
        intentar=recibir(prueba)
        if intentar:
            #conexion segunda
            iniciar()
    
    return

if __name__ == '__main__':
    from time import sleep
    main(True)
    sleep(5)
    cerrar()
