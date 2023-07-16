#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 20 20:18:34 2023

@author: kike
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy import linalg
import pandas as pd
import pylab


params = { 'backend': 'ps',
           'axes.labelsize': 24,
           'font.size': 28,
           'xtick.labelsize': 22,
           'ytick.labelsize': 22,
           'text.usetex': True }
pylab.rcParams.update(params)


data = np.genfromtxt('Gyroscope_calibration_biased.csv', dtype = float, delimiter=',').T

# data = np.genfromtxt('unbiased_test.csv', dtype = float, delimiter=',').T
plt.close('all')


gyroX = data[0]
gyroY = data[1]
gyroZ = data[2]

n = len(gyroX)

dt = 30/n
time = dt*np.arange(n)

print('Bias')
print(np.mean(data[:3], axis = 1))

print('Std')
print(np.std(data[:3], axis = 1))


# std = np.zeros([n-1, 3])
# for i in range(n-1):
#     std[i] = np.std(data[:3, :i]/int16bit_range*1000*degtorad, axis = 1)

# plt.figure()
# plt.plot(time[1:], std)




f, ax = plt.subplots(1, 2, constrained_layout = True, figsize = (12,5))
ax1 = ax[0]
ax1.plot(time, gyroX, '-o', label = '$GyroX$', markevery = 60)
ax1.plot(time, -gyroY, '-->', label = '$GyroY$', markevery = 60)
ax1.plot(time, -gyroZ, ':s', label = '$GyroZ$', markevery = 60)

ax1.set_xlabel('$t\>\mathrm{[s]}$')
ax1.set_ylabel('$\\omega\>\mathrm{[rad\>s^{-1}]}$')
ax1.grid(True)
ax1.set_xlim([0, time[-1]])
ax1.set_ylim([-0.3, 0.3])

data = np.genfromtxt('Gyroscope_calibration_unbiased.csv', dtype = float, delimiter=',').T


gyroX = data[0]
gyroY = data[1]
gyroZ = data[2]


dt = 30/len(gyroX)

time = dt*np.arange(len(gyroX))

ax2 = ax[1]
ax2.plot(time, gyroX, '-o', label = '$\omega_x$', markevery = 60)
ax2.plot(time, gyroY, '-->', label = '$\omega_x$', markevery = 60)
ax2.plot(time, gyroZ, ':s', label = '$\omega_x$', markevery = 60)
ax2.set_xlabel('$t\>\mathrm{[s]}$')
ax2.set_ylabel('$\\omega\>\mathrm{[rad\>s^{-1}]}$')
ax2.set_xlim([0, time[-1]])
ax2.set_ylim([-0.3, 0.3])
ax2.legend(fontsize = 20, ncol = 3)
plt.grid(True)
plt.show()





