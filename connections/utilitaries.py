#########################################################################
# CENTRALESUPELEC : ST5 VAC challenge week
#########################################################################

from __future__ import annotations

import struct
from typing import Optional, Tuple

from serial import Serial as SerialPort

def read_i16(f: SerialPort) -> int:
    return struct.unpack('<h', bytearray(f.read(2)))[0]

def read_i32(f: SerialPort) -> int:
    return struct.unpack('<l', bytearray(f.read(4)))[0]

def write_i16(f: SerialPort, value: int) -> None:
    f.write(struct.pack('<h', value))

def write_i32(f: SerialPort, value: int) -> None:
    f.write(struct.pack('<l', value))

def WaitACK(arduino: SerialPort) -> None:
    rep=b''
    while rep==b'':					# waiting acknowledgement of B2
        rep=arduino.readline()
    print(rep.decode())

def sendCmdi(arduino: SerialPort, cmd: bytes, arg1: int, arg2: int, arg3: int, arg4: int) -> None: # send command with 4 ints
    arduino.write(cmd)
    write_i16(arduino, arg1)
    write_i16(arduino, arg2)
    write_i16(arduino, arg3)
    write_i16(arduino, arg4)
    WaitACK(arduino)

def sendCmdl(arduino: SerialPort, cmd: bytes, arg1: int, arg2: int) -> None: # send command with 2 longs
    arduino.write(cmd)
    write_i32(arduino, arg1)
    write_i32(arduino, arg2)
    WaitACK(arduino)

def retrCmdi(arduino: SerialPort, cmd: bytes) -> Tuple[int, int, int, int]: # retrieve command with 4 ints
    arduino.write(cmd)
    val1=read_i16(arduino)
    val2=read_i16(arduino)
    val3=read_i16(arduino)
    val4=read_i16(arduino)
    return val1,val2,val3,val4
    WaitACK(arduino)

def retrCmdli(arduino: SerialPort, cmd: bytes) -> Optional[Tuple[int, int]]: # retrieve command with 1 long and 1 int
    """Réponse de Ret2li en binaire (long, int, int nul). None si elle n'arrive pas à temps."""
    arduino.write(cmd)
    reply = arduino.read(8)
    if len(reply) != 8:
        arduino.reset_input_buffer()  # sinon une fin de réponse en retard est lue comme un acquittement
        return None
    val1, val2, _ = struct.unpack('<lhh', reply)
    return val1, val2

def retrCmdl(arduino: SerialPort, cmd: bytes) -> Tuple[int, int]: # retrieve command with 2 longs
    arduino.write(cmd)
    val1=read_i32(arduino)
    val2=read_i32(arduino)
    return val1,val2
    WaitACK(arduino)
