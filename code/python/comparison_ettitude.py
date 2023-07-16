#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 14 18:17:44 2023

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




plt.close('all')

# for i in range(v.shape[0]):
#     data = np.genfromtxt('acc_cal_{:1.0f}.csv'.format(i+1), dtype = float, delimiter=',').T
#     data_acc = np.mean(data[3:6], axis = 1)/int16bit_range*4*g
#     v[i] = data_acc

   

# G = np.array([[0, 0, g], [g, 0, 0],[0, -g, 0],[0, g, 0]])
dt = 60e-3
data = np.genfromtxt('roll.csv', dtype = float, delimiter=',').T
time = dt*np.arange(data.shape[1])


YPR_ekf = data[0:3]*180/np.pi



YPR_tcf = data[3:]*180/np.pi


plt.figure(figsize = (8,6), constrained_layout = True)
plt.subplot(2,1,1)
plt.plot(time, YPR_ekf.T)
plt.legend(['$\phi$', '$\\theta$', '$\psi$'], ncol = 3, fontsize = 18)
plt.xlabel('$t~\mathrm{[s]}$')
plt.ylabel('$\mathrm{Angle~ [^\circ]}$', rotation = 90)
plt.grid(True)

plt.subplot(2,1,2)
plt.plot(time, YPR_tcf.T)
plt.xlabel('$t~\mathrm{[s]}$')
plt.ylabel('$\mathrm{Angle~ [^\circ]}$', rotation = 90)
plt.grid(True)
plt.show()



plt.figure(figsize = (10,6), constrained_layout = True)
plt.subplot(3,1,1)
plt.plot(time, YPR_ekf[0], 'k-o', markevery = 10)
plt.plot(time, YPR_tcf[0], '--s', markevery = 10)
plt.legend(['Kalman Filter', 'TRIAD + complementario'], ncol = 2, fontsize = 18)
plt.ylim([-20, 20])
plt.xlim([0, 10])
# plt.xlabel('$t~\mathrm{[s]}$')
plt.ylabel('$\phi~ \mathrm{[^\circ]}$', rotation = 90)
plt.grid(True)


plt.subplot(3,1,2)
plt.plot(time, YPR_ekf[1], 'k-o', markevery = 10)
plt.plot(time, YPR_tcf[1], '--s', markevery = 10)
plt.ylim([-20, 20])
plt.xlim([0, 10])
# plt.xticks([])
# plt.xlabel('$t~\mathrm{[s]}$')
plt.ylabel('$\\theta~ \mathrm{[^\circ]}$', rotation = 90)
plt.grid(True)

plt.subplot(3,1,3)
plt.plot(time, YPR_ekf[2], 'k-o', markevery = 10)
plt.plot(time, YPR_tcf[2], '--s', markevery = 10)
plt.ylim([0, 120])
plt.xlim([0, 10])
plt.xlabel('$t~\mathrm{[s]}$')
plt.ylabel('$\psi~ \mathrm{[^\circ]}$', rotation = 90)
plt.grid(True)
plt.show()

