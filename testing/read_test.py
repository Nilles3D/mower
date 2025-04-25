#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  read_test.py
#  
#  

import serial
import chardet
import time

def main(ser):
    while True:
        #grab whatever comes in
        lin=ser.readline()
        if lin!="":
            #attempt tp decode
            enc=chardet.detect(lin)['encoding']
            try:
                # ~ if enc!=None:
                    # ~ linS=lin.decode(enc)
                # ~ else:
                #forget about that gibberish
                linS=lin.decode()
                return linS[:-1]
            except:
                pass
    return ''

if __name__ == '__main__':
    ser=serial.Serial(port='/dev/serial0',
        baudrate=38400,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
    )
    
    cont=True
    t0=time.time()
    print('read_test start')
    
    while cont:
        try:
            red=main(ser)
    
            if red!='':
                t1=time.time()-t0
                print(round(t1,1), red)
                cont=(t1<=30)
    
        except KeyboardInterrupt:
            cont=False
            print('end')
