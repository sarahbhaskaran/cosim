from matplotlib import pyplot as plt
import os
import numpy as np

def graph_output(vehs=None):
    ''' Graph data in cosim/scripts/data from the last run of demo_node.py,
        saving the figures in cosim/scripts/figures
    '''

    dir = os.path.dirname(os.path.realpath(__file__))
    figure_path = os.path.join(dir, 'figures')
    if not os.path.exists(figure_path):
        os.makedirs(figure_path)
    # sumo_log.npy data was added in get_data_debug() of demo_node.py
    # [t, pos[0], pos[1], vel, self.space_gaps[veh].data, self.rel_vels[veh].linear.x]
    sumo = np.load(dir + '/data/sumo_log.npy')
    # Give generic names if the platoon is not provided
    if vehs == None:
        vehs = ['veh'+str(i) for i in range(sumo.shape[1])]
    if len(vehs) != sumo.shape[1]:
        print(f'{len(vehs)} vehicles in platoon names list, {sumo.shape[1]} in sumo data')
        vehs = ['veh'+str(i) for i in range(sumo.shape[1])]

    plt.figure()
    for i in range(sumo.shape[1]):
        plt.plot(sumo[:, i, 0], sumo[:, i, 1], label=vehs[i])
    plt.xlabel('time')
    plt.ylabel('xpos')
    plt.title('x position vs time')
    plt.legend()
    plt.savefig(figure_path+'/xpos_vs_time.png')

    plt.figure()
    for i in range(sumo.shape[1]):
        plt.plot(sumo[:, i, 0], sumo[:, i, 2], label=vehs[i])
    plt.xlabel('time')
    plt.ylabel('ypos')
    plt.title('y position vs time')
    plt.legend()
    plt.savefig(figure_path+'/ypos_vs_time.png')

    plt.figure()
    for i in range(sumo.shape[1]):
        plt.plot(sumo[:, i, 0], sumo[:, i, 3], label=vehs[i])
    plt.xlabel('time')
    plt.ylabel('vel')
    plt.legend()
    plt.title('Velocity vs time')
    plt.savefig(figure_path+'/vel_vs_time.png')

    plt.figure()
    for i in range(sumo.shape[1]):
        plt.plot(sumo[:, i, 0], sumo[:, i, 4], label=vehs[i])
    plt.xlabel('time')
    plt.ylabel('headway (m)')
    plt.legend()
    plt.title('Headway vs time')
    plt.savefig(figure_path+'/headway_vs_time.png')

    plt.figure()
    for i in range(sumo.shape[1]):
        plt.plot(sumo[:, i, 0], sumo[:, i, 4], label=vehs[i])
    plt.xlabel('time')
    plt.ylabel('relative velocity (m/s)')
    plt.legend()
    plt.title('Relative velocity of preceding car vs time')
    plt.savefig(figure_path+'/rel_vel_vs_time.png')

    ros = np.load(dir+'/data/sub_msgs.npy', allow_pickle=True)
    avs = [veh for veh in vehs if 'av' in veh]
    if len(avs) != ros.shape[0]:
        avs = [f'{str(i)}th av in platoon' for i in range(ros.shape[0])]

    plt.figure()
    for i in range(ros.shape[0]):
        plt.plot(ros[i, :, 0], ros[i, :, 1], label=avs[i])
    plt.xlabel('time')
    plt.ylabel('v_act.linear.x')
    plt.title('ROS msgs for velocity after accel dynamics')
    plt.legend()
    plt.savefig(figure_path+'/v_acts_vs_time.png')

    # Same figure? No, another plot with both on same figure
    plt.figure()
    cmd_vels = np.load(dir+'/data/cmd_vel_msgs.npy', allow_pickle=True)
    for i in range(cmd_vels.shape[0]):
        plt.plot(cmd_vels[i, :, 0], cmd_vels[i, :, 1], label=avs[i]+' cmd_vel')
    for i in range(ros.shape[0]):
        plt.plot(ros[i, :, 0], ros[i, :, 1], label=avs[i]+' v_act')
    plt.xlabel('time')
    plt.ylabel('velocity')
    plt.title('ROS msgs for cmd_vels before accel dynamics, v_acts after')
    plt.legend()
    plt.savefig(figure_path+'/v_acts_cmd_vels_vs_time.png')

    plt.figure()
    v_refs = np.load(dir+'/data/v_ref_msgs.npy', allow_pickle=True)
    for i in range(v_refs.shape[0]):
        plt.plot(v_refs[i, :, 0], v_refs[i, :, 1], label=avs[i])
    plt.xlabel('time')
    plt.ylabel('v_ref.linear.x')
    plt.title('ROS v_ref msgs (predicted acceleration)')
    plt.legend()
    plt.savefig(figure_path+'/v_refs_vs_time.png')


# plt.show()

