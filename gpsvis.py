#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  gpsvis.py
#  
#  Copyright 2024  <rpi32@rpi32>
#  
#  Transformar datos por visualizar y luego de nuevo por usar
#  

import webbrowser
import os
import shutil

arcRuta=os.path.dirname(os.path.abspath(__file__))+'/'
archivoGuia=arcRuta+'rutaSeguida.txt'
with open(archivoGuia,'r') as arcAct:
    archivoActual=arcAct.readline()
archivoExport=arcRuta+'rutas/datosPorVisualizer.txt'
archivoImport=arcRuta+'rutas/datosDeVisualizer.txt'

titulos=['type','latitude','longitude','name','color']

colores=[
    '#ff0000',
    '#ff8000',
    '#ffff00',
    '#80ff00',
    '#00ff00',
    '#00ff80',
    '#00ffff',
    '#0080ff',
    '#0000ff',
    '#8000ff',
    '#ff00ff'
    ]

url='https://www.gpsvisualizer.com/draw'

def rawToMap(rawFile: str,openHost=True):
    #lista de coordinates
    i=0
    cn=0
    mapPointList=[]
    with open(rawFile,'r') as arcSrc:
        for linea in arcSrc:
            d=linea[:-1]
            l=d.split()
            gps=['W',str(l[0]),str(l[1]),str(i),colores[cn]]
            mapPointList.append(gps)
            i+=1
            cn+=1
            cn=cn%len(colores)
    #lista por vis
    with open(archivoExport,'w') as ae:
        ae.write('\t'.join(titulos)+'\n')
        for gps in mapPointList:
            datu='\t'.join(gps)
            ae.write(datu+'\n')
    #haz modificaciones
    if openHost:
        webbrowser.open(url)
    return

def mapToRaw(mapFile: str):
    #lista modificado
    datPointList=[]
    with open(mapFile,'r') as arcSrc:
        next(arcSrc)
        for linea in arcSrc:
            d=linea[:-1]
            l=d.split('\t')
            if len(l)>1:
                ll=[float(l[1]),float(l[2])]
                datPointList.append(ll)
    #por mower
    arcViejo=archivoActual[:-4]+'v'+archivoActual[-4:]
    shutil.move(archivoActual,arcViejo)
    with open(archivoActual,'w') as ai:
        for dd in datPointList:
            datu=' '.join(str(ll) for ll in dd)
            ai.write(datu+'\n')
    
    return

if __name__ == '__main__':
    tiemActual=os.path.getmtime(archivoActual)
    tiemExport=os.path.getmtime(archivoExport)
    tiemImport=os.path.getmtime(archivoImport)
    if tiemExport<tiemActual:
        print('Actualizando '+archivoExport)
        rawToMap(archivoActual)
        print('Haz modificaciones quando quieres')
    elif tiemExport<tiemImport:
        print('Actualizando '+archivoImport)
        mapToRaw(archivoImport)
        print('Listo por Mower')
    else:
        print('Nada por hacer')
