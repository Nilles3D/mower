#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  gnss3.py
#  
#  Copyright 2025  <rpi31@rpi31>
#  
#  Simpler reader of UBX
#  credit https://github.com/semuconsulting/pyubx2/blob/master/examples/gnssapp.py
#  

from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser
from queue import Empty, Queue
from threading import Event, Thread
from time import sleep

from pyrtcm import RTCMMessage, RTCMMessageError, RTCMParseError
from serial import Serial

from pyubx2 import (
    RTCM3_PROTOCOL,
    UBX_PROTOCOL,
    UBXMessageError,
    UBXParseError,
    UBXReader,
    ubxtypes_decodes as ubd
)

CONNECTED = 1
#fixType=-1

class GNSSBarebones:
    
    def __init__(
        self, port: str, baudrate: int, timeout: float, stopevent: Event, verbose: bool, **kwargs
    ):
        """
        Constructor.

        :param str port: serial port e.g. "/dev/ttyACM1"
        :param int baudrate: baudrate
        :param float timeout: serial timeout in seconds
        :param Event stopevent: stop event
        """

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.stopevent = stopevent
        self.stream = None
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.sep = 0
        self.heading = 0 #headMot?
        self.fixType = 0
        self.verbose = verbose
        
        print("gnss3    GNSSBarebones initialized")

    def __enter__(self):
        """
        Context manager enter routine.
        """
        print("gnss3    GNSSBarebones entered")

        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        """
        Context manager exit routine.

        Terminates app in an orderly fashion.
        """

        self.stop()
        print("gnss3    GNSSBarebones exited")
        
    def run(self):
        """
        Run GNSS reader/writer.
        """

        self.stream = Serial(self.port, self.baudrate, timeout=self.timeout)
        self.stopevent.clear()

        read_thread = Thread(
            target=self._read_loop,
            args=(
                self.stream,
                self.stopevent,
                self.verbose
            ),
            daemon=True,
        )
        read_thread.start()
        print("gnss3    GNSS Barebones is running")

    def stop(self):
        """
        Stop GNSS reader/writer.
        """
        self.stopevent.set()
        if self.stream is not None:
            self.stream.close()
        print("gnss3    GNSS Barebones stopped")
        
    def _read_loop(self, stream: Serial, stopevent: Event, verbose=False):
        """
        THREADED
        Reads and parses incoming GNSS data from the receiver.

        :param Serial stream: serial stream
        :param Event stopevent: stop event
        """
        print("gnss3    GNSS read_loop start")
        
        gdata0=None
        pdata0=None

        ubr = UBXReader(
            stream, protfilter=(UBX_PROTOCOL | RTCM3_PROTOCOL)
        )
        while not stopevent.is_set():
            try:
                if stream.in_waiting:
                    # ~ print(stream.readline())
                    _, parsed_data = ubr.read()
                    if parsed_data:
                        # extract current navigation solution
                        self._extract_coordinates(parsed_data, verbose)
                        # extract current GPS fix solution
                        self._extract_fixType(parsed_data, verbose)

                        if verbose:
                            # if it's an RXM-RTCM message, show which RTCM3 message
                            # it's acknowledging and whether it's been used or not.""
                            # ~ if parsed_data.identity == "RXM-RTCM":
                                # ~ nty = (
                                    # ~ f" - {parsed_data.msgType} "
                                    # ~ f"{'Used' if parsed_data.msgUsed > 0 else 'Not used'}"
                                # ~ )
                            # ~ else:
                                # ~ nty = ""

                            # ~ if self.idonly:
                                # ~ print(f"gnss3   GNSS>> {parsed_data.identity}{nty}")
                            # ~ else:
                                # ~ print(f"gnss3   GNSS+> {parsed_data}")
                            
                            #packet visualization
                            gdata1=[self.lat, self.lon, self.alt, self.heading, self.fixType]
                            if gdata1!=gdata0:
                                print("gnss3    lat, lon, alt, heading, fix")
                                print(f"gnss3   {gdata1}")
                                gdata0=gdata1
                            # ~ pdata1=parsed_data
                            # ~ if pdata1!=pdata0:
                                # ~ print(f"gnss3   parsed: {pdata1}")
                                # ~ pdata0=pdata1

            except (
                UBXMessageError,
                UBXParseError,
                RTCMMessageError,
                RTCMParseError,
            ) as err:
                print(f"gnss3   Error parsing data stream {err}")
                continue
        print("gnss3    GNSS read_loop stop")

    def _extract_coordinates(self, parsed_data: object, verbose=False):
        """
        Extract current navigation solution from UBX message.

        :param object parsed_data: parsed UBX navigation message
        """

        if hasattr(parsed_data, "lat"):
            self.lat = parsed_data.lat
        if hasattr(parsed_data, "lon"):
            self.lon = parsed_data.lon
        if hasattr(parsed_data, "alt"):
            self.alt = parsed_data.alt
        if hasattr(parsed_data, "hMSL"):  # UBX hMSL is in mm
            self.alt = parsed_data.hMSL / 1000
        if hasattr(parsed_data, "sep"):
            self.sep = parsed_data.sep
        if hasattr(parsed_data, "hMSL") and hasattr(parsed_data, "height"):
            self.sep = (parsed_data.height - parsed_data.hMSL) / 1000
        if hasattr(parsed_data, "heading"):
            self.heading = parsed_data.heading

    def _extract_fixType(self, parsed_data: object, verbose=False):
        """
        Extract state of GNSS solution
        """
        if hasattr(parsed_data, "fixType"):
            if self.fixType!=parsed_data.fixType:
                self.fixType=parsed_data.fixType
                if verbose:
                    print(f"gnss3   Fix Type now {ubd.FIXTYPE[self.fixType]}")
    
    def get_position(self) -> tuple:
        '''
        simplified get_coordinates
        '''
        return (self.lat, self.lon)
        

def gnssIniciar(eventPara: Event, port='/dev/serial0', baud=38400, outTime=3, verbose=False):
    print("gnss3    Iniciando GNSS")
    
    #iniciar sustantivo
    arp = ArgumentParser(
        formatter_class=ArgumentDefaultsHelpFormatter,
    )
    arp.add_argument(
        "-P", "--port", required=False, help="Serial port", default=port
    )
    arp.add_argument(
        "-B", "--baudrate", required=False, help="Baud rate", default=baud, type=int
    )
    arp.add_argument(
        "-T", "--timeout", required=False, help="Timeout in secs", default=outTime, type=float
    )

    args = arp.parse_args()
    
    stop_event = eventPara #Event()
    
    #enviar por correr
    gna = GNSSBarebones(
        args.port,
        int(args.baudrate),
        float(args.timeout),
        stop_event,
        verbose=verbose,
    )
    gna.run()
    
    print("gnss3    Incindido GNSS")
    return gna
    

if __name__ == '__main__':
    
    stop_event = Event()

    try:
        print("Starting GNSS reader\n")
        newGna=gnssIniciar(stop_event, verbose=True)            
        sleep(20)
        stop_event.set()
            
    except KeyboardInterrupt:
        stop_event.set()
        print("Terminated by user")
        
    finally:
        print("End GNSS reader")
