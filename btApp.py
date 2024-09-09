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

import bluetooth as bt

#bt setup
btReady=False
server_sock=bt.BluetoothSocket(bt.RFCOMM)
port=22
server_sock.bind(("",port))
server_sock.listen(1)
client_sock,address=server_sock.accept()
print("btApp    Conexion realizada con: ",address)
btReady=True

from time import sleep

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

def cerrar():
    print("btApp    cerrando...")
    btReady=False
    client_sock.close()
    server_sock.close()
    print("btApp    cerrado")

def main():
    try:
        while len(client_sock.getpeername())>0:
            print("wait")
            recvdata=client_sock.recv(256)
            #on receipt
            recCode=recvdata.decode()
            print(recCode)
            if len(recCode)>1:
                recVar=int(recCode[:1])
                recNum=int(recCode[1:])
            else:
                recVar=0
                recNum=0
            #use
            print(recVar, recNum)
            if recVar==powerCode:
                powerVal=recNum
            elif recVar==biasCode:
                biasVal=recNum
            elif recVar==steerCode:
                steerVal=recNum+biasVal
            else:
                print("btApp    %i,%i",recVar,recNum)
            #exit attempt
            if recVar==closeCode:
                cerrar()
                return 0
    except Exception as err:
        print(err)
    except KeyboardInterrupt:
        print("btApp    saliendo")
    finally:
        cerrar()
    return 0


if __name__ == '__main__':
    main()
