# -*- coding: utf-8 -*-
#%%

import serial

READY_CMD = '1'
CHECK_AVAILABLE_CMD = '2'
SET_BASE_CMD = '3'
SET_FREQ_CMD = '4'
SET_AMPLITUDE_CMD = '5'
SET_DURATION_CMD = '6'
COMPUTE_LUT_CMD = '7'
BEGIN_CMD = '8'
SET_KP_CMD = '9'
SET_KD_CMD = 'A'
SET_KI_CMD = 'B'
SET_OFFSET_CMD = 'C'
ACK = 'a'
NACK = 'n'

class TRECS:
    def pitch(self, degrees):
        #(-0.3656 * analogRead(SENSOR_PIN)) + 185.64
        val = (degrees-185.64)/(-.3656)
        return val
    def flush(self):
        while self.ser.in_waiting:
            self.ser.read()
    def sendCmd(self, cmd):
        if type(cmd) != bytes:
            cmd = cmd.encode()
        s = cmd+b'\n'
        assert len(s) == len(cmd)+1
        self.ser.write(s)
    def readResp(self):
        resp = self.ser.readline()
        return resp[:-2].decode()
    def isAvailable(self):
        self.sendCmd(CHECK_AVAILABLE_CMD)
        return self.readResp() == ACK
    def setBase(self, base):
        base = int(self.pitch(base))
        assert base == base&0xFFFF
        self.sendCmd(SET_BASE_CMD)
        assert self.readResp() == ACK
        s = b''.join([((base&0xFF00)>>8).to_bytes(1, 'little'), (base&0xFF).to_bytes(1, 'little')])
        self.sendCmd(s)
        assert int(self.readResp()) == base
        assert self.readResp() == ACK
    def setFreq(self, freq):
        freq *= 10
        freq = int(freq)
        assert freq == freq&0xFFFF
        self.sendCmd(SET_FREQ_CMD)
        assert self.readResp() == ACK
        s = b''.join([((freq&0xFF00)>>8).to_bytes(1, 'little'), (freq&0xFF).to_bytes(1, 'little')])
        self.sendCmd(s)
        assert int(self.readResp()) == freq
        assert self.readResp() == ACK
    def setAmp(self, amp):
        amp = int(self.pitch(amp))
        assert amp == amp&0xFFFF
        self.sendCmd(SET_AMPLITUDE_CMD)
        assert self.readResp() == ACK
        s = b''.join([((amp&0xFF00)>>8).to_bytes(1, 'little'), (amp&0xFF).to_bytes(1, 'little')])
        self.sendCmd(s)
        assert int(self.readResp()) == amp
        assert self.readResp() == ACK
    def setDuration(self, periods_measure, periods_wait):
        assert periods_measure == periods_measure&0xFF
        assert periods_wait == periods_wait&0xFF
        self.sendCmd(SET_DURATION_CMD)
        assert self.readResp() == ACK
        s = ''.join([chr(periods_measure), chr(periods_wait)])
        self.sendCmd(s)
        assert int(self.readResp()) == periods_measure
        assert int(self.readResp()) == periods_wait
        assert self.readResp() == ACK
    def setKp(self, Kp):
        Kp = int(Kp*1000)
        assert Kp == Kp&0xFFFF
        self.sendCmd(SET_KP_CMD)
        assert self.readResp() == ACK
        s = b''.join([((Kp&0xFF00)>>8).to_bytes(1, 'little'), (Kp&0xFF).to_bytes(1, 'little')])
        self.sendCmd(s)
        assert int(self.readResp()) == Kp
        assert self.readResp() == ACK
    def setKd(self, Kd):
        Kd = int(Kd*1000)
        assert Kd == Kd&0xFFFF
        self.sendCmd(SET_KD_CMD)
        assert self.readResp() == ACK
        s = b''.join([((Kd&0xFF00)>>8).to_bytes(1, 'little'), (Kd&0xFF).to_bytes(1, 'little')])
        self.sendCmd(s)
        assert int(self.readResp()) == Kd
        assert self.readResp() == ACK
    def setKi(self, Ki):
        Ki = int(Ki*1000)
        assert Ki == Ki&0xFFFF
        self.sendCmd(SET_KI_CMD)
        assert self.readResp() == ACK
        s = b''.join([((Ki&0xFF00)>>8).to_bytes(1, 'little'), (Ki&0xFF).to_bytes(1, 'little')])
        self.sendCmd(s)
        assert int(self.readResp()) == Ki
        assert self.readResp() == ACK
    def setOffset(self, offset):
        offset = int(offset)
        assert offset == offset&0xFFFF
        self.sendCmd(SET_OFFSET_CMD)
        assert self.readResp() == ACK
        s = b''.join([((offset&0xFF00)>>8).to_bytes(1, 'little'), (offset&0xFF).to_bytes(1, 'little')])
        self.sendCmd(s)
        assert int(self.readResp()) == offset
        assert self.readResp() == ACK
    def computeLut(self):
        self.sendCmd(COMPUTE_LUT_CMD)
        assert self.readResp() == ACK
        values = []
        while True:
            resp = self.readResp()
            if resp == ACK:
                break
            values.append(int(resp))
        return values
    def begin(self):
        self.sendCmd(BEGIN_CMD)
        assert self.readResp() == ACK # Acked command
        assert self.readResp() == ACK # Finished taking measurements
        values = []
        while True:
            resp = self.readResp()
            if resp == ACK:
                break
            values.append(int(resp))
        return values
    def __init__(self, port):
        self.ser = serial.Serial(port=port, baudrate=115200)
        resp = self.readResp()
        if resp != READY_CMD:
            self.__del__()
            assert False
        self.sendCmd(READY_CMD)
    def __del__(self):
        self.ser.close()

#%%
      
import numpy as np
from matplotlib import pyplot as plt

offset = 70
Kp = .05
Kd = 0
Ki = 0

base = 0
frequency = 2 # Hz; Maximum frequency: 72
amplitude = 1
periods_measure = 4
periods_wait = 8

try:
    trecs = TRECS('COM4')
    assert trecs.isAvailable()
    trecs.setOffset(offset)
    trecs.setKp(Kp)
    trecs.setKd(Kd)
    trecs.setKi(Ki)
    trecs.setBase(base)
    trecs.setAmp(amplitude)
    trecs.setFreq(frequency)
    trecs.setDuration(periods_measure, periods_wait)
    input_values_p = trecs.computeLut()
    input_values = []
    for i in range(periods_measure):
        input_values += input_values_p
    output_values = trecs.begin()
    assert len(output_values) == len(input_values)
    plt.plot(range(len(input_values)), input_values, color='blue')
    plt.figure()
    plt.plot(range(len(output_values)), output_values, color='red')
finally:
    del trecs