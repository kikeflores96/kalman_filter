#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 13 20:17:43 2023

@author: kike
"""

import numpy as np
import Kalman_EKF_mod_output as km




t = 100/1000




w = np.array([7.98753855e-04, -2.49584207e-03 , 1.81871684e-03])

a = np.array([2.80537553e+00, 5.84316071e-02,  9.78944766e+00])

m = np.array([-2.79830467e+01,  1.23371929e+01, -2.82501149e+01])



ekf = km.System()



ekf.predict(w, t)

ekf.update(a, m)