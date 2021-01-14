#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Dec  5 12:15:13 2020

__author__ = sebastian.tilaguy@gmail.com
__version__ = "1.0"
__maintainer__ = "Sebastian Tilaguy"
__email__ = "sebastian.tilaguy@gmail.com"
__status__ = "Development"
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


data = pd.read_csv('ekf_modified.txt', sep="\t", header=0,index_col=None)
# [x y yaw vx vy W x_N y_E yaw y_ICR_r y_ICR_l x_ICR_v]
m,n = data.shape
print(data)

Fs = 10
Ts = 1/Fs
t = np.linspace(0,m*Ts,m)

rad2deg = 180/np.pi

plt.figure(1)
plt.subplot(211)
plt.plot(data['y_ICR_r'].values)
plt.plot(data['y_ICR_l'].values)
plt.plot(data['x_ICR_v'].values)
plt.legend(['Y_ICRr','Y_ICRl','X_ICR'])
plt.subplot(212)
plt.plot(data['yaw_imu'].values*rad2deg,'r')
plt.plot(data['yaw'].values*rad2deg,'--b')
plt.plot(data['yaw_e'].values*rad2deg,'.k')
plt.legend(['mag','model','kalman'])

# plt.figure(2)
# plt.plot(data['old_x'].values,data['old_y'].values)

plt.figure(3)
plt.plot(data['x_N'].values,data['y_E'].values,'r')
plt.plot((data['x'].values),(data['y'].values),'.b')
plt.plot(data['x_e'].values,data['y_e'].values,'--k')
plt.legend(['model','GPS','kalman'])

plt.figure(4)
plt.subplot(311)
plt.plot(data['ref_r'].values*0.064/15,'k')
plt.plot(data['wr'].values*0.064)
plt.plot(data['wr_f'].values*0.064,'--')
plt.plot(data['vr'].values,'--g')
plt.legend(['ref','vr_enc','vr_f','vr_ekf'])
plt.subplot(312)
plt.plot(data['ref_l'].values*0.064/15,'k')
plt.plot(data['wl'].values*0.064)
plt.plot(data['wl_f'].values*0.064,'--')
plt.plot(data['vl'].values,'--g')
plt.legend(['ref','vl_enc','vl_f','vl_ekf'])
plt.subplot(313)
plt.plot((data['ref_r'].values*0.064/15)-(data['ref_l'].values*0.064/15),'k')
plt.plot((data['wr'].values*0.064)-(data['wl'].values*0.064))
plt.plot((data['wr_f'].values*0.064)-(data['wl_f'].values*0.064))
plt.plot(data['vr'].values-data['vl'].values)
plt.legend(['ref','enc','filt','ekf'])

plt.show()

