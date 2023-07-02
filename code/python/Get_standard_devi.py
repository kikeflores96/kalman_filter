#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 30 17:01:01 2023

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


data = np.genfromtxt('std.csv', dtype = float, delimiter=',').T

# data = np.genfromtxt('unbiased_test.csv', dtype = float, delimiter=',').T
# plt.close('all')


gyroX = data[0]
gyroY = data[1]
gyroZ = data[2]



accX = data[3]
accY = data[4]
accZ = data[5]


magX = data[6]
magY = data[7]
magZ = data[8]


mod_Acc = np.sqrt(accX**2 + accY**2 + accZ**2)

mod_mag = np.sqrt(magX**2 + magY**2 + magZ**2)



accX = data[3]/mod_Acc
accY = data[4]/mod_Acc
accZ = data[5]/mod_Acc


magX = data[6]/mod_mag
magY = data[7]/mod_mag
magZ = data[8]/mod_mag

print('GyroX_var = {:8.7e} [rad/s]²'.format(np.std(gyroX)**2))
print('GyroY_var = {:8.7e} [rad/s]²'.format(np.std(gyroY)**2))
print('GyroZ_var = {:8.7e} [rad/s]²'.format(np.std(gyroZ)**2))

print('AccX_var = {:8.7e} [rad/s]²'.format(np.std(accX)**2))
print('AccY_var = {:8.7e} [rad/s]²'.format(np.std(accY)**2))
print('AccZ_var = {:8.7e} [rad/s]²'.format(np.std(accZ)**2))

print('MagX_var = {:8.7e} [rad/s]²'.format(np.std(magX)**2))
print('MagY_var = {:8.7e} [rad/s]²'.format(np.std(magY)**2))
print('MagZ_var = {:8.7e} [rad/s]²'.format(np.std(magZ)**2))
