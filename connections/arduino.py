#########################################################################
# CENTRALESUPELEC : ST5 VAC challenge week
#########################################################################

from __future__ import annotations

import time
from typing import Optional, Tuple

import serial

from connections import utils
from connections.utilitaries import WaitACK, retrCmdi, retrCmdli, sendCmdi


class Arduino:

    def __init__(self, portnr: str) -> None:
        self.serialport = serial.Serial(port=portnr, baudrate=utils.BAUD_RATE, timeout=utils.SERIAL_TIMEOUT)

    #   connection to Arduino
    def connect(self) -> bool:
        if self.serialport.is_open == False:
            self.serialport.open()
        rep = b' '  # emptying serial link
        while rep != b'':
            rep = self.serialport.readline()
        print("Connecting to Arduino")

        time.sleep(utils.ARDUINO_BOOT_DELAY)  # waiting for the Arduino to be initialised

        self.serialport.write(b'A22')  # requesting connection with simple 'OK' acknowledgement
        rep = self.serialport.readline()
        if rep.split()[0] == b'OK':
            self.serialport.write(b'I0')
            WaitACK(self.serialport)
            print(rep.decode())
            return True
        else:
            return False

    #   disconnection
    def disconnect(self) -> None:
        self.serialport.write(b'a')  # disconnection
        self.serialport.close()  # closing serial link

    #   stop motors
    def Stop(self) -> None:
        sendCmdi(self.serialport, b'C', 0, 0, 0, 0)

    def SendSpeeds(self, fl: int, fr: int, rr: int, rl: int) -> None:
        sendCmdi(self.serialport, b'C', fr, fl, rr, rl)

    #   moving forward
    def Advance(self, c1: int, c2: int) -> None:
        sendCmdi(self.serialport, b'C', c1, c2, c1, c2)

    #   mise en route des moteurs en sens inverse
    def Back(self, c1: int, c2: int) -> None:
        sendCmdi(self.serialport, b'C', -c1, -c2, -c1, -c2)

    #   mise en route des moteurs pour tourner à gauche
    def TurnLeft(self, c1: int, c2: int) -> None:
        sendCmdi(self.serialport, b'C', c1, -c2, c1, -c2)

    #   mise en route des moteurs pour tourner à droite
    def TurnRight(self, c1: int, c2: int) -> None:
        sendCmdi(self.serialport, b'C', -c1, c2, -c1, c2)

    #   mise en route de la protection infra rouge
    def IrOn(self) -> None:
        self.serialport.write(b'I1')
        WaitACK(self.serialport)

    #   recuperation de la vitesse des moteurs
    def GetSpeed(self) -> Tuple[int, int, int, int]:
        vit1, vit2, vit3, vit4 = retrCmdi(self.serialport, b'P')
        return vit1, vit2, vit3, vit4

    #   distance mesuree par le capteur ultrason en cm (None si l'Arduino ne repond pas)
    def GetDistance(self) -> Optional[int]:
        reply = retrCmdli(self.serialport, b'S')
        return None if reply is None else reply[1]
