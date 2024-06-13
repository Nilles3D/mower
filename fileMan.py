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
    #returns empty string if folder does not exist
    
    if not carpeta[-1:]=="/":
        carpeta += "/"
        print(f'prefixMatch Continuando con {carpeta}')
    
    if not dirChec(carpeta, True):
        print(f'prefixMatch No era un {carpeta} por buscar. Ahora si.')
        return ''
    
    carpObj=os.scandir(carpeta)
    suff=0
    for entry in carpObj:
        if entry.is_file() and entry.name.startswith(titulo):
            suff+=1
    if suff>0:
        archivoNombre=titulo+' '+str(suff)
    else:
        archivoNombre=titulo
    
    archivoRuta=carpeta+archivoNombre
    
    return archivoRuta

def dirChec(dirPuesto: str, crear = False):
    
    if os.path.exists(dirPuesto):
        existando = os.path.isdir(dirPuesto)
    elif crear:
        os.mkdir(dirPuesto)
        print(f'dirChec Directorio {dirPuesto} creada')
        existand = True
    else:
        existando = False
    
    return existando

if __name__ == '__main__':
    print('___START fileMan___')
    pc=os.path.dirname(os.path.abspath(__file__))+'/'
    print(prefixMatch(pc,"mower"))
    print(dirChec(pc+"logs",True))
    print('___END fileMan___')
