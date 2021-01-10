#!/usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np

currentDir = '/home/i2rlab/catkin_ws/src/mrs_taskplan/node/'

#traj1_a_x, traj1_a_y = np.loadtxt(currentDir + 'traj1a.txt' , delimiter = ' ', unpack = True)
traj1_d_x, traj1_d_y = np.loadtxt(currentDir + 'traj1d.txt' , delimiter = ' ', unpack = True)
#plt.plot(traj1_a_x, traj1_a_y, 'r:', label = 'actual')
plt.plot(traj1_d_x, traj1_d_y, 'b:', label = 'desired')


#traj2_a_x, traj2_a_y = np.loadtxt(currentDir + 'traj2_a.txt' , delimiter = ' ', unpack = True)
#traj2_d_x, traj2_d_y = np.loadtxt(currentDir + 'traj2_d.txt' , delimiter = ' ', unpack = True)
#plt.plot(traj2_a_x, traj2_a_y, 'r:', label = 'actual')
#plt.plot(traj2_d_x, traj2_d_y, 'b:', label = 'desired')

#traj2_a_x, traj2_a_y = np.loadtxt(currentDir + 'traj3a.txt' , delimiter = ' ', unpack = True)
#traj2_d_x, traj2_d_y = np.loadtxt(currentDir + 'traj3d.txt' , delimiter = ' ', unpack = True)
#plt.plot(traj2_a_x, traj2_a_y, 'r:', label = 'actual')
#plt.plot(traj2_d_x, traj2_d_y, 'b:', label = 'desired')


#traj3_a_x, traj3_a_y = np.loadtxt(currentDir + 'traj3_a.txt' , delimiter = ' ', unpack = True)
#traj3_d_x, traj3_d_y = np.loadtxt(currentDir + 'traj3_d.txt' , delimiter = ' ', unpack = True)
#plt.plot(traj3_a_x, traj3_a_y, 'r:', label = 'actual')
#plt.plot(traj3_d_x, traj3_d_y, 'b:', label = 'desired')

#with warnings.catch_warnings():
#  warnings.simplefilter("ignore")
#  traj4_a_x, traj4_a_y = np.loadtxt(currentDir + 'traj4_a.txt' , delimiter = ' ', unpack = True)
#  traj4_d_x, traj4_d_y = np.loadtxt(currentDir + 'traj4_d.txt' , delimiter = ' ', unpack = True)
#plt.plot(traj4_a_x, traj4_a_y, 'r:', label = 'actual')
#plt.plot(traj4_d_x, traj4_d_y, 'b:', label = 'desired')

plt.legend()
plt.ylabel('y position')
plt.xlim(0, 3.68)
plt.ylim(0, 2.9)
plt.show()
