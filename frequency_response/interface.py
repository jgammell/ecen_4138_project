# -*- coding: utf-8 -*-
"""
Created on Mon Nov 23 14:42:06 2020

@author: jgamm
"""

#%%

import numpy as np
from matplotlib import pyplot as plt

center = 100
amplitude = 50
settling_periods = 10
# measurement_periods = 15
frequency = 1000

import serial

try:
    TRECS = serial.Serial('COM7', 115200, timeout=1)
    while TRECS.readline() != b'r\r\n':
        pass
    print('TRECS ready')
    TRECS.write(b'.')
    while TRECS.readline() != b'A\r\n':
        pass
    print('Sending data')
    TRECS.write(b'%d\n'%(center))
    TRECS.write(b'%d\n'%(amplitude))
    TRECS.write(b'%d\n'%(settling_periods))
    TRECS.write(b'%d\n'%(frequency))
    print('Done')
    for i in range(7):
        print(TRECS.readline().decode())
    vals = []
    while True:
        val = TRECS.readline()
        val = val.decode()[:-2]
        if val == 'Done':
            break
        else:
            vals.append(np.int16(val))
        
finally:
    TRECS.close()
    plt.plot(range(len(vals)), vals)