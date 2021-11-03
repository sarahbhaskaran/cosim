import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import os

dir = os.path.realpath(os.path.dirname('__file__'))

plt.figure()
sumo = np.load(dir + '/data/sumo_log.npy')
plt.plot(sumo[:, 0], sumo[:, 1], label='ego')
plt.plot(sumo[:, 0], sumo[:, 4], label='lead')
plt.xlabel('time')
plt.ylabel('pos')
plt.legend()

plt.figure()
sumo = np.load(dir + '/data/sumo_log.npy')
plt.plot(sumo[:, 0], sumo[:, 2], label='ego')
plt.plot(sumo[:, 0], sumo[:, 5], label='lead')
plt.xlabel('time')
plt.ylabel('velocity (SUMO)')
plt.legend()

plt.figure()
subs = np.load(dir+'/data/pub_msgs.npy')
plt.plot(subs[:, 0], subs[:, 1], label='vel')
pubs = np.load(dir+'/data/pub_msgs.npy')
plt.plot(pubs[:, 0], pubs[:, 1], label='cmd_vel')
plt.xlabel('time')
plt.ylabel('ROS msg')
plt.legend()
plt.show()
