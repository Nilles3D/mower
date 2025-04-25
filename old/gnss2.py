#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  gnss2.py
#  
#  Copyright 2024  <rpi31@rpi31>
#  


#credit https://github.com/semuconsulting/pyubx2/blob/master/examples/gnssapp.py

"""
pygnssutils - gnssapp.py

*** FOR ILLUSTRATION ONLY - NOT FOR PRODUCTION USE ***

Skeleton GNSS application which continuously receives, parses and prints
NMEA, UBX or RTCM data from a receiver until the stop Event is set or
stop() method invoked. Assumes receiver is connected via serial USB or UART1 port.

The app also implements basic methods needed by certain pygnssutils classes.

Optional keyword arguments:

- sendqueue - any data placed on this Queue will be sent to the receiver
  (e.g. UBX commands/polls or NTRIP RTCM data). Data must be a tuple of 
  (raw_data, parsed_data).
- idonly - determines whether the app prints out the entire parsed message,
  or just the message identity.
- enableubx - suppresses NMEA receiver output and substitutes a minimum set
  of UBX messages instead (NAV-PVT, NAV-SAT, NAV-DOP, RXM-RTCM).
- showhacc - show estimate of horizonal accuracy in metres (if available).

Created on 27 Jul 2023

:author: semuadmin
:copyright: SEMU Consulting © 2023
:license: BSD 3-Clause
"""
# pylint: disable=invalid-name, too-many-instance-attributes

from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser
from queue import Empty, Queue
from threading import Event, Thread
from time import sleep

from pynmeagps import NMEAMessageError, NMEAParseError
from pyrtcm import RTCMMessage, RTCMMessageError, RTCMParseError
from serial import Serial

from pyubx2 import (
    NMEA_PROTOCOL,
    RTCM3_PROTOCOL,
    UBX_PROTOCOL,
    UBXMessage,
    UBXMessageError,
    UBXParseError,
    UBXReader,
    ubxtypes_core as ubt,
    ubxtypes_decodes as ubd
)

CONNECTED = 1
#fixType=-1


class GNSSSkeletonApp:
    """
    Skeleton GNSS application which communicates with a GNSS receiver.
    """

    def __init__(
        self, port: str, baudrate: int, timeout: float, stopevent: Event, **kwargs
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
        self.sendqueue = kwargs.get("sendqueue", None)
        self.idonly = kwargs.get("idonly", True)
        self.enableubx = kwargs.get("enableubx", False)
        self.showhacc = kwargs.get("showhacc", False)
        self.stream = None
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.sep = 0
        self.heading = 0 #headMot?
        self.fixType = 0
        self.hAcc = 0
        
        print("gnss2    GNSS Skeleton initialized")

    def __enter__(self):
        """
        Context manager enter routine.
        """
        print("gnss2    GNSSSkeletonApp entered")

        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        """
        Context manager exit routine.

        Terminates app in an orderly fashion.
        """

        self.stop()
        print("gnss2    GNSSSkeletonApp exited")

    def run(self, verbose=False):
        """
        Run GNSS reader/writer.
        """

        self.enable_ubx(self.enableubx)

        self.stream = Serial(self.port, self.baudrate, timeout=self.timeout)
        self.stopevent.clear()

        read_thread = Thread(
            target=self._read_loop,
            args=(
                self.stream,
                self.stopevent,
                self.sendqueue,
                verbose,
            ),
            daemon=True,
        )
        read_thread.start()
        print("gnss2    GNSS Skeleton is running")

    def stop(self):
        """
        Stop GNSS reader/writer.
        """
        self.stopevent.set()
        if self.stream is not None:
            self.stream.close()
        print("gnss2    GNSS Skeleton stopped")

    def _read_loop(self, stream: Serial, stopevent: Event, sendqueue: Queue, verbose=False):
        """
        THREADED
        Reads and parses incoming GNSS data from the receiver,
        and sends any queued output data to the receiver.

        :param Serial stream: serial stream
        :param Event stopevent: stop event
        :param Queue sendqueue: queue for messages to send to receiver
        """
        print("gnss2    GNSS read_loop start")
        print(f"gnss2   verbose = {verbose}")
        
        gdata0=[]
        pdata0=None

        ubr = UBXReader(
            stream, protfilter=(NMEA_PROTOCOL | UBX_PROTOCOL | RTCM3_PROTOCOL)
        )
        while not stopevent.is_set():
            try:
                if stream.in_waiting:
                    _, parsed_data = ubr.read()
                    if parsed_data:
                        if isinstance(parsed_data, str) and verbose:
                            if parsed_data.find('UNKNOWN PROTOCOL') > 0:
                                print(f"!!! {parsed_data}")
                        # extract current navigation solution
                        self._extract_coordinates(parsed_data, verbose)
                        # extract current GPS fix solution
                        self._extract_fixType(parsed_data, verbose)

                        if verbose:
                            # if it's an RXM-RTCM message, show which RTCM3 message
                            # it's acknowledging and whether it's been used or not.""
                            if hasattr(parsed_data, "identity"):
                                if parsed_data.identity == "RXM-RTCM":
                                    nty = (
                                        f" - {parsed_data.msgType} "
                                        f"{'Used' if parsed_data.msgUsed > 0 else 'Not used'}"
                                    )
                                else:
                                    nty = ""
                            
                            if hasattr(parsed_data, "header"):
                                uky = ", header = "+str(decode(parsed_data.header))
                            else:
                                uky = ""
                                
                            if self.idonly:
                                print(f"gnss2   GNSS>> {parsed_data.identity}{nty}")
                            else:
                                print(f"gnss2   GNSS+> {parsed_data}{uky}")
                                # ~ sleep(4)
                            
                            #packet visualization
                            gdata1=[self.lat, self.lon, self.alt, self.heading, self.fixType]
                            # ~ print('gdata')
                            # ~ print(gdata1)
                            if gdata1!=gdata0:
                                print("gnss2    lat, lon, alt, heading, fix")
                                print(f"gnss2   {gdata1}")
                                gdata0=gdata1
                            # ~ pdata1=parsed_data
                            # ~ if pdata1!=pdata0:
                                # ~ print(f"gnss2   parsed: {pdata1}")
                                # ~ pdata0=pdata1

                # send any queued output data to receiver
                self._send_data(ubr.datastream, sendqueue, verbose)

            except (
                UBXMessageError,
                UBXParseError,
                NMEAMessageError,
                NMEAParseError,
                RTCMMessageError,
                RTCMParseError,
            ) as err:
                print(f"gnss2   Error parsing data stream {err}")
                continue
        print("gnss2    GNSS read_loop stop")

    def _extract_coordinates(self, parsed_data: object, verbose=False):
        """
        Extract current navigation solution from NMEA or UBX message.

        :param object parsed_data: parsed NMEA or UBX navigation message
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
        if self.showhacc and hasattr(parsed_data, "hAcc"):  # UBX hAcc is in mm
            unit = 1 if parsed_data.identity == "PUBX00" else 1000
            if verbose:
                if self.hAcc!=parsed_data.hAcc:
                    print(f"gnss2   Estimated horizontal accuracy: {(parsed_data.hAcc / unit):.3f} m")
                    self.hAcc=parsed_data.hAcc
        if hasattr(parsed_data, "headMot"): #direction of motion, headVeh = point direction
            self.heading = parsed_data.headMot 

    def _extract_fixType(self, parsed_data: object, verbose=False):
        """
        Extract state of GNSS solution
        """
        if hasattr(parsed_data, "fixType"):
            if self.fixType!=parsed_data.fixType:
                self.fixType=parsed_data.fixType
                if verbose:
                    print(f"gnss2   Fix Type now {ubd.FIXTYPE[self.fixType]}")
    
    def _send_data(self, stream: Serial, sendqueue: Queue, verbose=False):
        """
        Send any queued output data to receiver.
        Queue data is tuple of (raw_data, parsed_data).

        :param Serial stream: serial stream
        :param Queue sendqueue: queue for messages to send to receiver
        """

        if sendqueue is not None:
            try:
                while not sendqueue.empty():
                    data = sendqueue.get(False)
                    raw, parsed = data
                    if verbose:
                        source = "NTRIP>>" if isinstance(parsed, RTCMMessage) else "GNSS<<"
                        if self.idonly:
                            print(f"gnss2   Sending {source} {parsed.identity}")
                        else:
                            print(f"gnss2   Sending {parsed}")
                    stream.write(raw)
                    sendqueue.task_done()
            except Empty:
                pass
            except TypeError:
                print("!!!gnss2    Could not unpack non-iterable")
                print(data)

    def enable_ubx(self, enable: bool):
        """
        Enable UBX output and suppress NMEA.

        :param bool enable: enable UBX and suppress NMEA output
        """

        layers = 1
        transaction = 0
        cfg_data = []
        for port_type in ("USB", "UART1"):
            cfg_data.append((f"CFG_{port_type}OUTPROT_NMEA", not enable))
            cfg_data.append((f"CFG_{port_type}OUTPROT_UBX", enable))
            cfg_data.append((f"CFG_MSGOUT_UBX_NAV_PVT_{port_type}", enable))
            cfg_data.append((f"CFG_MSGOUT_UBX_NAV_SAT_{port_type}", enable * 4))
            cfg_data.append((f"CFG_MSGOUT_UBX_NAV_DOP_{port_type}", enable * 4))
            cfg_data.append((f"CFG_MSGOUT_UBX_RXM_RTCM_{port_type}", enable))

        msg = UBXMessage.config_set(layers, transaction, cfg_data)
        self.sendqueue.put((msg.serialize(), msg))
        
        st="" if enable else "not "
        print(f"gnss2    GNSS UBX is {st}enabled")

    def get_coordinates(self) -> tuple:
        """
        Return current receiver navigation solution.
        (method needed by certain pygnssutils classes)

        :return: tuple of (connection status, lat, lon, alt and sep)
        :rtype: tuple
        """

        return (CONNECTED, self.lat, self.lon, self.alt, self.sep)

    def get_position(self) -> tuple:
        '''
        simplified get_coordinates
        '''
        return (self.lat, self.lon)
        
    def set_event(self, eventtype: str):
        """
        Create event.
        (stub method needed by certain pygnssutils classes)

        :param str eventtype: name of event to create
        """

        # create event of specified eventtype
    
    def _put_message(self, msg):
        '''
        Put a command on the send_queue
        :msg UBX message
        '''
        self.sendqueue.put((msg.serialize(),msg))
        
    def cfg_WT(self, in_autoDirPinPolOff: int, in_wtFactor: int):
        """
        Basic values for RLM
        """
        msg=UBXMessage("CFG","CFG-ESFWT",ubt.SET,
            combineTicks=1,
            autoDirPinPolOff=in_autoDirPinPolOff, #1=Off
            wtFactor=in_wtFactor #um
        )
        # ~ print("gnss2    Configuring WT")
        # ~ print(msg)
        self._put_message(msg)
        print("gnss2    WT configured")
        
        return
    
    def put_wheelTick(self, ticksLeft: int, dirLeft: bool, ticksRight: int, dirRight: bool, timestamp):
        """
        Forward is True
        """
        msgClass="ESF"
        msgID="ESF-MEAS"
        #tick measurements
        tL=ticksLeft.to_bytes(23,"big")
        dL=(int(not dirLeft)).to_bytes(1,"big")
        xL=bytes(bytearray(tL+dL))
        dR=(int(not dirRight)).to_bytes(1,"big")
        tR=ticksRight.to_bytes(23,"big")
        xR=bytes(bytearray(tR+dR))
        #bytearray vs bitarray nomenclature is unclear
        msg=UBXMessage(msgClass,msgID,ubt.GET,
            numMeas=2,
            dataField_01=xL, #X024
            dataType_01=8, #U06
            dataField_02=xR,
            dataType_02=9,
            timeTag=timestamp
        )
        self._put_message(msg)
        
        return
        

def gnssIniciar(eventPara: Event, port='/dev/serial0', baud=115200, outTime=3, verbose=False):
    print("gnss2    Iniciando GNSS")
    
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
    send_queue = Queue()
    stop_event = eventPara #Event()
    
    gna = GNSSSkeletonApp(
        args.port,
        int(args.baudrate),
        float(args.timeout),
        stop_event,
        sendqueue=send_queue,
        idonly=False,
        enableubx=True,
        showhacc=True,
    )
    gna.run(verbose=verbose)
    
    print("gnss2    Incindido GNSS")
    return gna, send_queue



if __name__ == "__main__":
    if True:
        print('de casa')
        arp = ArgumentParser(
            formatter_class=ArgumentDefaultsHelpFormatter,
        )
        arp.add_argument(
            "-P", "--port", required=False, help="Serial port", default="/dev/serial0" #ttyACM1
        )
        arp.add_argument(
            "-B", "--baudrate", required=False, help="Baud rate", default=38400, type=int
        )
        arp.add_argument(
            "-T", "--timeout", required=False, help="Timeout in secs", default=3, type=float
        )

        args = arp.parse_args()
        send_queue = Queue()
        stop_event = Event()

        try:
            print("Starting GNSS reader/writer...\n")
            with GNSSSkeletonApp(
                args.port,
                int(args.baudrate),
                float(args.timeout),
                stop_event,
                sendqueue=send_queue,
                idonly=False,
                enableubx=True,
                showhacc=True,
            ) as gna:
                gna.run(verbose=True)
                while True:
                    sleep(1)
                stop_event.set()
                print(gna.baudrate)
                
        except KeyboardInterrupt:
            stop_event.set()
            print("Terminated by user")
    else:
        try:
            print('\nby def')
            gnssPara=Event()
            gnaObj, sendQ=gnssIniciar(gnssPara,verbose=True)
            sleep(0.2)
            tickScale=105 #um
            # ~ gnaObj.cfg_WT(1,tickScale)
            sleep(0.2)
            import datetime
            misaDataTima=datetime.datetime.now()
            print(misaDataTima)
            mdt=int(misaDataTima.microsecond/1000)
            print(mdt)
            # ~ gnaObj.put_wheelTick(1,True,1,False,mdt)
            sleep(60)
            gnssPara.set()
            gnaObj.stop()
            print(gnaObj.baudrate)
        except KeyboardInterrupt:
            stop_event.set()
            print("Finishing")
    
