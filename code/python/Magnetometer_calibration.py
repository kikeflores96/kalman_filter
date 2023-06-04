#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Feb 27 18:04:18 2021

@author: kike
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy import linalg
import pylab

params = { 'backend': 'ps',
           'axes.labelsize': 20,
           'font.size': 20,
           'xtick.labelsize': 16,
           'ytick.labelsize': 16,
           'text.usetex': True }
pylab.rcParams.update(params)

def fitEllipsoid(magX, magY, magZ):
    a1 = magX ** 2
    a2 = magY ** 2
    a3 = magZ ** 2
    a4 = 2 * np.multiply(magY, magZ)
    a5 = 2 * np.multiply(magX, magZ)
    a6 = 2 * np.multiply(magX, magY)
    a7 = 2 * magX
    a8 = 2 * magY
    a9 = 2 * magZ
    a10 = np.ones(len(magX)).T
    D = np.array([a1, a2, a3, a4, a5, a6, a7, a8, a9, a10])

    # Eqn 7, k = 4
    C1 = np.array([[-1, 1, 1, 0, 0, 0],
                   [1, -1, 1, 0, 0, 0],
                   [1, 1, -1, 0, 0, 0],
                   [0, 0, 0, -4, 0, 0],
                   [0, 0, 0, 0, -4, 0],
                   [0, 0, 0, 0, 0, -4]])

    # Eqn 11
    S = np.matmul(D, D.T)
    S11 = S[:6, :6]
    S12 = S[:6, 6:]
    S21 = S[6:, :6]
    S22 = S[6:, 6:]

    # Eqn 15, find eigenvalue and vector
    # Since S is symmetric, S12.T = S21
    tmp = np.matmul(np.linalg.inv(C1), S11 - np.matmul(S12, np.matmul(np.linalg.inv(S22), S21)))
    eigenValue, eigenVector = np.linalg.eig(tmp)
    u1 = eigenVector[:, np.argmax(eigenValue)]

    # Eqn 13 solution
    u2 = np.matmul(-np.matmul(np.linalg.inv(S22), S21), u1)

    # Total solution
    u = np.concatenate([u1, u2]).T

    Q = np.array([[u[0], u[5], u[4]],
                  [u[5], u[1], u[3]],
                  [u[4], u[3], u[2]]])

    n = np.array([[u[6]],
                  [u[7]],
                  [u[8]]])

    d = u[9]

    return Q, n, d

def main():
    # data = np.genfromtxt('magnetometer_data.csv', dtype = float, delimiter=',')
    data = np.genfromtxt('mag_cal_02.csv', dtype = float, delimiter=',')

    # magX = data[:, 6] * 0.00080
    # magY = data[:, 7] * 0.00080
    # magZ = data[:, 8] * 0.00080
    int16bit_range = 32767.5
    
    
    magX = data[:, 6]/int16bit_range*4800 
    magY = data[:, 7]/int16bit_range*4800
    magZ = data[:, 8]/int16bit_range*4800
    
    mag_x0 = 23.87
    mag_y0 = 0.06375
    mag_z0 = 39.984
    
    mod_mag = np.linalg.norm(np.array([mag_x0,mag_y0, mag_z0]))
    
    magX /= mod_mag
    magY /= mod_mag
    magZ /= mod_mag
    
    # magX = data[:, 6]
    # magY = data[:, 7]
    # magZ = data[:, 8]

    plt.close('all')
    fig1, axes = plt.subplots(1, 2, figsize = (9, 5), constrained_layout = True, subplot_kw={'projection': '3d', 'box_aspect': (1,1,1)})
    # axes[0] = plt.axes(projection='3d')
    plt.suptitle('$\mathrm{Calibracion\>del\>magnetometro}$')
    ax1 = axes[0]
    ax1.set_box_aspect(aspect = (1,1,1.4))
    ax1.scatter(magX, magY, magZ, s=5, color='r')
    ax1.set_xlabel('$\\tilde{h}_x$')
    ax1.set_ylabel('$\\tilde{h}_y$')
    ax1.set_zlabel('$\\tilde{h}_z$')
    
    # plot unit sphere
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones(np.size(u)), np.cos(v))
    ax1.plot_wireframe(x, y, z, rstride=10, cstride=10, alpha=0.5)
    ax1.plot_surface(x, y, z, alpha=0.3, color='b')

    Q, n, d = fitEllipsoid(magX, magY, magZ)

    Qinv = np.linalg.inv(Q)
    b = -np.dot(Qinv, n)
    Ainv = np.real(1 / np.sqrt(np.dot(n.T, np.dot(Qinv, n)) - d) * linalg.sqrtm(Q))

    print("A_inv: ")
    print(Ainv)
    print()
    print("b")
    print(b)
    print()

    calibratedX = np.zeros(magX.shape)
    calibratedY = np.zeros(magY.shape)
    calibratedZ = np.zeros(magZ.shape)

    totalError = 0
    for i in range(len(magX)):
        h = np.array([[magX[i], magY[i], magZ[i]]]).T
        hHat = np.matmul(Ainv, h-b)
        calibratedX[i] = hHat[0]
        calibratedY[i] = hHat[1]
        calibratedZ[i] = hHat[2]
        mag = np.dot(hHat.T, hHat)
        err = (mag[0][0] - 1)**2
        totalError += err
    print("Total Error: %f" % totalError)

    # fig2 = plt.figure(2)
    # axes[1] = plt.axes(projection='3d')
    ax2 = axes[1]
    ax2.scatter(calibratedX, calibratedY, calibratedZ, s=1, color='r')
    ax2.set_xlabel('$\\tilde{h}_x$')
    ax2.set_ylabel('$\\tilde{h}_y$')
    ax2.set_zlabel('$\\tilde{h}_z$')
    ax2.set_box_aspect(aspect = (1,1,1))

    # plot unit sphere
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones(np.size(u)), np.cos(v))
    ax2.plot_wireframe(x, y, z, rstride=10, cstride=10, alpha=0.5)
    ax2.plot_surface(x, y, z, alpha=0.3, color='b')
    # plt.tight_layout()
    plt.grid(True)
    plt.show()
    
    return magX, magY, magZ



if __name__ == '__main__':
    magx, magy, magz = main()