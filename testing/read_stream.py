#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  read_stream.py
#  
#  Copyright 2025  <rpi31@rpi31>
#  
#  https://github.com/semuconsulting/pyubx2

from serial import Serial
from pyubx2 import UBXReader, UBX_PROTOCOL
import time

cont=True
t0=time.time()
t1e=10 #runtime, seconds

with Serial('/dev/serial0', 38400, timeout=3) as stream:
  print('start read_stream')
  while cont:
    try:
        print('Entering reader at '+str(round(time.time()-t0,1))+'. Will continue until data is received.')
        
        ubr = UBXReader(stream, protfilter=UBX_PROTOCOL)
        raw_data, parsed_data = ubr.read()
        
        if parsed_data is not None:
            print('data!')
            print(parsed_data)
            # ~ if hasattr(parsed_data, 'lat'):
                # ~ print(parsed_data.lat)
                # ~ print(parsed_data.lon)
                # ~ print(parsed_data.motHeading)
                
    except Exception as e:
        print('err: '+str(e))
        
    except (KeyboardInterrupt) as e:
        cont=False
    
    finally:
        cont=time.time()-t0<=t1e
        print('end or timeout at '+str(round(time.time()-t0,1)))
