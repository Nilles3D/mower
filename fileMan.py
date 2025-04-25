#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  fileMan.py
#  
#  Copyright 2024  <rpi31@rpi31>
#  
#  all typical file and directory ops
#  

import os

def prefixMatch(carpeta,titulo):
    #given directory/ to search for files that start with a title
    #returns name with a numerical suffix if >0
    
    carpObj=os.scandir(carpeta)
    suff=0
    for entry in carpObj:
        if entry.is_file() and entry.name.startswith(titulo):
            suff+=1
    if suff>0:
        archivoNombre=titulo+' '+str(suff)
    else:
        archivoNombre=titulo
    
    archivoRuta=carpeta+titulo
    
    return archivoRuta

if __name__ == '__main__':
    print('___START fileMan___')
    
    print('___END fileMan___')
