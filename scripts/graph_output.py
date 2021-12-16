from matplotlib import pyplot as plt
import os
import numpy as np

def graph_output(vehs=None):
    dir = os.path.realpath(os.path.dirname('__file__'))
    sumo = np.load(dir + '/data/sumo_log.npy')
    # Give generic names if the platoon is not provided
    if vehs == None:
        vehs = ['veh'+str(i) for i in range(sumo.shape[1])]
    if len(vehs) != sumo.shape[1]:
        print(f'{len(vehs)} vehicles in platoon names list, {sumo.shape[1]} in sumo data')
        vehs = ['veh'+str(i) for i in range(sumo.shape[1])]

    plt.figure()
    for i in range(1, sumo.shape[1]):
        plt.plot(sumo[:, i, 0], sumo[:, i, 1], label=vehs[i])
    plt.xlabel('time')
    plt.ylabel('xpos')
    plt.title('x position vs time')
    plt.legend()
    plt.savefig(dir+'/xpos_vs_time.png')

    plt.figure()
    for i in range(sumo.shape[1]):
        plt.plot(sumo[:, i, 0], sumo[:, i, 2], label=vehs[i])
    plt.xlabel('time')
    plt.ylabel('ypos')
    plt.title('y position vs time')
    plt.legend()
    plt.savefig(dir+'/ypos_vs_time.png')

    plt.figure()
    for i in range(sumo.shape[1]):
        plt.plot(sumo[:, i, 0], sumo[:, i, 3], label=vehs[i])
    plt.xlabel('time')
    plt.ylabel('vel')
    plt.legend()
    plt.title('Velocity vs time')
    plt.savefig(dir+'/vel_vs_time.png')

    plt.figure()
    for i in range(sumo.shape[1]):
        plt.plot(sumo[:, i, 0], sumo[:, i, 4], label=vehs[i])
    plt.xlabel('time')
    plt.ylabel('headway (m)')
    plt.legend()
    plt.title('Headway vs time')
    plt.savefig(dir+'/headway_vs_time.png')

    plt.figure()
    for i in range(sumo.shape[1]):
        plt.plot(sumo[:, i, 0], sumo[:, i, 4], label=vehs[i])
    plt.xlabel('time')
    plt.ylabel('relative velocity (m/s)')
    plt.legend()
    plt.title('Relative velocity of preceding car vs time')
    plt.savefig(dir+'/rel_vel_vs_time.png')

graph_output(['lead', 'av1'])
# plt.show()

