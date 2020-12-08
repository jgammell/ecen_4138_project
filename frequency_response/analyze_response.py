#%%
# -*- coding: utf-8 -*-
"""
Created on Tue Dec  8 01:43:37 2020

@author: jgamm
"""

from matplotlib import pyplot as plt
import numpy as np
import os
import pickle
from scipy.optimize import curve_fit

inputs = []
responses = []
frequencies = [.1, .2, .3, .4, .5, .6, .7, .8, .9, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

datapath = os.getcwd()+'/data/'
for frequency in frequencies:
    with open(datapath+'%2.1fHz.pickle'%(frequency), 'rb') as F:
        data = pickle.load(F)
        inputs.append(data['Input'])
        responses.append(data['Output'])

inputs = np.array([inp-np.mean(inp) for inp in inputs])
responses = np.array([resp-np.mean(resp) for resp in responses])

inputs_params = []
responses_params = []
for inp, resp, freq in zip(inputs, responses, frequencies):
    time = np.linspace(0, 2/freq, len(inp))
    def f(x, A, phi):
        return A*np.cos(2*np.pi*freq*x+phi)
    params, _ = curve_fit(f, time, inp)
    inputs_params.append(params)
    params, _ = curve_fit(f, time, resp)
    responses_params.append(params)

magnitude_response = np.array([20*np.log10(np.abs(resp[0]/inp[0])) for resp, inp in zip(responses_params, inputs_params)])
phase_response = np.array([resp[1]-inp[1] for resp, inp in zip(responses_params, inputs_params)])
(fig, ax) = plt.subplots(2, 1, figsize=(16, 12), sharex=True)
ax[0].set_xscale('log')
ax[1].set_xlabel('Frequency (Hz)')
ax[0].set_ylabel('Magnitude (dB)')
ax[1].set_ylabel('Phase ($2\pi$Hz)')
ax[0].plot(frequencies, magnitude_response, color='blue')
ax[1].plot(frequencies, phase_response, color='blue')
fig.savefig(datapath+'/../figures/response.png')
