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
            # ~ enc=chardet.detect(lin)['encoding']
            # ~ try:
                # ~ if enc!=None:
                    # ~ linS=lin.decode(enc)
                # ~ else:
                # ~ #forget about that gibberish
                    # ~ linS=lin.decode()
                # ~ return linS[:-1]
            # ~ except:
                # ~ pass
            return lin
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
            red=''
            if ser.in_waiting>0:
                red=main(ser)
            
            t1=time.time()-t0
            
            if red!='':
                print(round(t1,1), red[:100])
            
            cont=(t1<=30)
            
            ser.flush()
                
        except (KeyboardInterrupt) as e:
            cont=False
            ser.close()
            print('end')
