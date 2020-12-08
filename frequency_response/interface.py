# -*- coding: utf-8 -*-
#%%

import serial
import time

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
BEGIN_STABILIZE_CMD = 'D'
ACK = 'a'
NACK = 'n'

class TRECS:
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
        base = int((base-185.64)/(-.3656))
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
        amp = int(amp/.3656)
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
            values.append(-0.3656 * int(resp) + 185.64)
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
            values.append(-0.3656 * int(resp) + 185.64)
        return values
    def beginStabilize(self, pitch, duration):
        pitch = int((pitch-185.64) / (-.3656))
        assert pitch&0xFFFF == pitch
        self.sendCmd(BEGIN_STABILIZE_CMD)
        assert self.readResp() == ACK
        s = b''.join([((pitch&0xFF00)>>8).to_bytes(1, 'little'), (pitch&0xFF).to_bytes(1, 'little')])
        print('Sending pitch: %d'%(pitch))
        self.sendCmd(s)
        assert int(self.readResp()) == pitch
        assert self.readResp() == ACK
        print('Starting stabilization')
        pitches = []
        t0 = time.time()
        while time.time()-t0 < duration:
            val = int(self.readResp())
            pitches.append(-0.3656 * val + 185.64)
        self.sendCmd(NACK)
        while True:
            val = self.readResp()
            try:
                _ = int(val)
            except:
                assert val == ACK
                break
        return pitches
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
import os
import pickle

# 11.28 degrees is roughly vertical

offset = 300
Kp = 0
Kd = 0
Ki = 0

base = 11.28
frequency = 10 # Hz; Maximum frequency: 72
amplitude = 30
periods_measure = 2
periods_wait = 4

try:
    trecs = TRECS('COM4')
    assert trecs.isAvailable()
    trecs.setOffset(offset)
    trecs.setBase(base)
    trecs.setFreq(frequency)
    trecs.setAmp(amplitude)
    trecs.setDuration(periods_measure, periods_wait)
    trecs.setKp(Kp)
    trecs.setKd(Kd)
    trecs.setKi(Ki)
    input_pitches0 = trecs.computeLut()
    output_pitches = trecs.begin()
finally:
    del trecs
    input_pitches = []
    for i in range(periods_measure):
        input_pitches += input_pitches0
    (fig, ax) = plt.subplots(2, 1, figsize=(16, 12), sharex=True)
    ax[0].plot(np.linspace(0, periods_measure/frequency, len(input_pitches)), input_pitches)
    ax[1].plot(np.linspace(0, periods_measure/frequency, len(output_pitches)), output_pitches)
    with open(os.getcwd()+'/data/%2.1fHz.pickle'%(frequency), 'wb') as F:
        pickle.dump({'Input': input_pitches, 'Output': output_pitches}, F)