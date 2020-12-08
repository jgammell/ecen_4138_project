#%%
# -*- coding: utf-8 -*-
"""
Created on Mon Dec  7 14:14:15 2020

@author: jgamm
"""

import serial
import numpy as np
from matplotlib import pyplot as plt
plt.rcParams.update({'font.size': 22})
import os
import pickle

TRECS = serial.Serial('COM4', 115200)
line = TRECS.readline()
print(line)
ref_times = np.zeros(128)
for i in range(128):
    line = TRECS.readline()
    ref_times[i] = int(line)
    print(ref_times[i])
line = TRECS.readline()
print(line)
fast_times = np.zeros(128)
for i in range(128):
    line = TRECS.readline()
    fast_times[i] = int(line)
    print(fast_times[i])
line = TRECS.readline()
print(line)
slow_times = np.zeros(128)
for i in range(128):
    line = TRECS.readline()
    slow_times[i] = int(line)
    print(slow_times[i])
TRECS.close()

mean_ref = np.mean(ref_times)
fast_times -= mean_ref
slow_times -= mean_ref
(fig, ax) = plt.subplots(1, 1, figsize=(16, 12))
ax.plot(np.arange(128), ref_times, color='grey', label='Measurement overhead')
ax.plot(np.arange(128), fast_times, color='red', label='Duration of improved function')
ax.plot(np.arange(128), slow_times, color='blue', label='Duration of original function')
ax.set_xlabel('Measurement number')
ax.set_ylabel('Duration (µs)')
ax.legend()
fig.savefig(os.getcwd()+'/comparison_plot.pdf')
with open(os.getcwd()+'/comparison_data.pickle', 'wb') as F:
    pickle.dump({'Reference times': ref_times, 'Improved times': fast_times, 'Original times': slow_times}, F)