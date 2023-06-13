#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 26 13:38:19 2023

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



int16bit_range = 32767.5
g = 9.81
v = np.zeros([4, 3])


# for i in range(v.shape[0]):
#     data = np.genfromtxt('acc_cal_{:1.0f}.csv'.format(i+1), dtype = float, delimiter=',').T
#     data_acc = np.mean(data[3:6], axis = 1)/int16bit_range*4*g
#     v[i] = data_acc

   

# G = np.array([[0, 0, g], [g, 0, 0],[0, -g, 0],[0, g, 0]])



for i in range(v.shape[0]):
    data = np.genfromtxt('acc_cal_{:1.0f}_b.csv'.format(i+1), dtype = float, delimiter=',').T
    data_acc = np.mean(data[3:6], axis = 1)
    v[i] = data_acc

   

G = np.array([[0, 0, g], [-g, 0, 0],[0, -g, 0],[0, g, 0]])



m1 = np.array([[1,  G[0, 2]], 
               [1 , G[1, 2]]])

b1 = np.array([[v[0,2]],
               [v[1,2]]])
    
a1 = np.matmul(np.linalg.inv(m1), b1)    

bz, kz = a1

m2 = np.array([[1, G[0, 1], G[0, 2]], 
               [1, G[1, 1], G[1, 2]], 
               [1, G[2, 1], G[2, 2]] ])

b2 = np.array([[v[0,1]],
               [v[1,1]],
               [v[2,1]]])
    
a2 = np.matmul(np.linalg.inv(m2), b2)    

by, ky, kyzx = a2

m3 = np.array([[1, G[0, 0], G[0, 1],  -G[0,2]], 
               [1, G[1, 0], G[1, 1],  -G[1,2]], 
               [1, G[2, 0], G[2, 1],  -G[2,2]], 
               [1, G[3, 0], G[3, 1],  -G[3,2]] ])

b3 = np.array([[v[0,0]],
               [v[1,0]],
               [v[2,0]],
               [v[3,0]]])
    
a3 = np.matmul(np.linalg.inv(m3), b3)  

bx, kx, kxyz, kxzy = a3


azx = kyzx/ky
ayz = kxyz/kx
azy = kxzy/kx + ayz*azx

K = np.array([[kx[0], 0, 0],
              [0, ky[0], 0],
              [0, 0, kz[0]]])

A = np.array([[1, -ayz[0],  azy[0]],
              [0,    1,    -azx[0]],
              [0,    0,          1]])

B = np.array([bx[0], by[0], bz[0]]).T


v_c = np.zeros_like(v)

for i in range(v.shape[0]):
    v_c[i] = np.matmul(A, np.matmul(np.linalg.inv(K), (v[i].T - B)))

    
#%%

plt.close('all')

n=3


f, ax = plt.subplots(n, 2, constrained_layout = True, figsize = (14, 8.5))
ax[0,0].set_title('$\mathrm{No calibration}$')
ax[0,1].set_title('$\mathrm{Calibrated}$')
for i in range(n):
    if i==2:
        data = np.genfromtxt('acc_cal_{:1.0f}_b.csv'.format(i+2), dtype = float, delimiter=',').T
    else:
        data = np.genfromtxt('acc_cal_{:1.0f}_b.csv'.format(i+1), dtype = float, delimiter=',').T
    

    # data = np.genfromtxt('acc_cal_1.csv', dtype = float, delimiter=',').T
    data_acc = data[3:6]
    
    time = 50e-3*np.arange(data_acc.shape[1])
    
    
    cal_data = np.zeros_like(data_acc)
    
    
    cal_data = np.matmul(A, np.matmul(np.linalg.inv(K), (data_acc.T - B).T))
    
    
    
    
    
    ax1 = ax[i, 0]
    
    ax1.plot(time, data_acc[0], '-o', label = '$a_x$', markevery = 60)
    ax1.plot(time, data_acc[1], '-->', label = '$a_y$', markevery = 60)
    ax1.plot(time, data_acc[2], ':s', label = '$a_z$', markevery = 60)
    if i==0:
        ax1.legend(ncol = 3, fontsize = 16, loc = 'best')
    
    ax1.set_ylabel('$a\>\mathrm{[m\>s^{-2}]}$')
    ax1.grid(True)
    ax1.set_xlim([0, time[-1]])
    ax1.set_ylim([-3, 11])
    
    
    ax2 = ax[i, 1]
    ax2.plot(time, cal_data[0], '-o', label = '$GyroX$', markevery = 60)
    ax2.plot(time, cal_data[1], '-->', label = '$GyroY$', markevery = 60)
    ax2.plot(time, cal_data[2], ':s', label = '$GyroZ$', markevery = 60)
    # ax2.set_xlabel('$t\>\mathrm{[s]}$')
    # ax2.set_ylabel('$a\>\mathrm{[m\>s^{-2}]}$')
    ax2.set_yticklabels([])
    if i!=2:
        # ax2.set_yticks([])
        ax1.set_xticklabels([])
        ax2.set_xticklabels([])
    else:
        ax1.set_xlabel('$t\>\mathrm{[s]}$')
        ax2.set_xlabel('$t\>\mathrm{[s]}$')
    
    ax2.set_xlim([0, time[-1]])
    ax2.set_ylim([-3, 11])
    
    ax2.grid(True)
    plt.show()

