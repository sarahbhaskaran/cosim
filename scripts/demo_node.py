#!/usr/bin/env python
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import numpy as np
import os
import rospy
import traci
import time
import pandas as pd
from args import parse_args
from utils import fs_accel, idm_accel


class SumoHostNode:
    def __init__(self, args):
        self.args = args
        gui = False
        if gui:
            sumo_bin = os.path.dirname(os.path.realpath(os.environ['SUMO_HOME'])) + '/sumo-gui'
        else:
            sumo_bin = os.environ['SUMO_HOME']
        self.node_path = os.path.dirname(os.path.realpath(__file__))
        sumo_cfg = os.path.join(self.node_path, 'I24/I24.sumo.cfg')
        self.min_gap = 0
        self.platoon_set = args.platoon
        self.platoon_names = ['lead'] + [veh + str(i+1) for i, veh in enumerate(self.platoon_set)]
        print('Running with platoon', self.platoon_names)
        self.avs = [veh for veh in self.platoon_names if 'av' in veh]
        self.dx = .05
        sumo_cmd = [sumo_bin,
                    '-c', sumo_cfg,
                    '--seed', '42',
                    '--step-length', str(self.dx),
                    '--step-method.ballistic',
                    '--collision.mingap-factor', str(self.min_gap),
                    '--collision.action', 'none']
        traci.start(sumo_cmd, label='sim')
        self.kernel = traci.getConnection('sim')

        self.car_len = 0
        self.starting_positions = []
        self.add_vehicles_debug()
        self.kernel.simulationStep()

        # self.pub = rospy.Publisher('/v_act', Twist, queue_size=0)
        # self.sub = rospy.Subscriber('/vel', Twist, self.callback)

        self.ego_vel_pubs = {veh: rospy.Publisher(veh+'/vel', Twist, queue_size=0) for veh in self.avs}
        self.space_gap_pubs = {veh: rospy.Publisher(veh+'/lead_dist', Float64, queue_size=0) for veh in self.avs}
        self.rel_vel_pubs = {veh: rospy.Publisher(veh+'/rel_vel', Twist, queue_size=0) for veh in self.avs}
        self.acc_pubs = {av: rospy.Publisher(av+'/msg_467', Point, queue_size=0) for av in self.avs}
        self.v_act_subs = {veh: rospy.Subscriber(veh+'/v_act', Twist, callback=self.v_act_callback_gen(veh)) for veh in self.avs}
        self.v_ref_subs = {veh: rospy.Subscriber(veh+'/v_ref', Twist, callback=self.v_ref_callback_gen(veh)) for veh in self.avs}

        self.pub_datas = {veh: [] for veh in self.avs}
        self.sub_datas = {veh: [] for veh in self.avs}
        self.v_ref_datas = {veh: [] for veh in self.avs}
        self.vels = {veh: Twist() for veh in self.platoon_names}
        self.rel_vels = {veh: Twist() for veh in self.platoon_names}
        self.space_gaps = {veh: Float64() for veh in self.platoon_names}
        self.accs = {veh: Point() for veh in self.avs}
        for acc in self.accs.values():
            acc.y = 100
        self.v_acts = {veh: Twist() for veh in self.platoon_names}
        self.v_refs = {veh: Twist() for veh in self.platoon_names}

        # Make lead vel have the velocity of a real car
        lead_vel_csv = os.path.join(self.node_path, 'car_vel.csv')
        self.lead_vels = pd.read_csv(lead_vel_csv).iterrows()

        self.sumo_data = []
        self.t0 = time.time()
        self.rate = rospy.Rate(20)
        self.t = time.time() - self.t0
        for veh in self.avs:
            self.pub_datas[veh].append([self.t, self.vels[veh].linear.x])
        # for t in range(4320):
        for t in range(500):
            self.rate.sleep()
            self.sumo_data.append(self.get_data_debug())

            curr_lead_vel = self.get_lead_vel(t)
            # curr_lead_vel = 15
            self.vels['lead'].linear.x = curr_lead_vel
            self.v_acts['lead'].linear.x = curr_lead_vel

            for veh in self.platoon_names:
                self.kernel.vehicle.setSpeed(veh, self.v_acts[veh].linear.x)
                if 'idm' in veh:
                    self.get_next_vel(veh, 'idm')
                else:
                    print('v act', self.v_acts[veh].linear.x)

            self.kernel.simulationStep()

            self.t = time.time() - self.t0

            # Get info about current stuff
            self.get_sim_state()
            # Publish info that RL controller takes as input
            for veh in self.avs:
                self.ego_vel_pubs[veh].publish(self.vels[veh])
                self.space_gap_pubs[veh].publish(self.space_gaps[veh])
                self.rel_vel_pubs[veh].publish(self.rel_vels[veh])
                self.acc_pubs[veh].publish(self.accs[veh])
                print(self.vels[veh].linear.x, 'vel')
                print(self.rel_vels[veh].linear.x, 'rel vel')
                print(self.space_gaps[veh].data, 'gap')
                self.pub_datas[veh].append([self.t, self.vels[veh].linear.x])

    def get_next_vel(self, veh, mode):
        curr_vel = self.vels[veh].linear.x
        ahead_veh = self.platoon_names[self.platoon_names.index(veh)-1]
        ahead_vel = self.vels[ahead_veh].linear.x
        gap = self.space_gaps[veh].data
        if mode == 'fs':
            accel = fs_accel(gap, curr_vel, ahead_vel)
        elif mode == 'idm':
            accel = idm_accel(gap, curr_vel, ahead_vel)
        self.vels[veh].linear.x = curr_vel + self.dx*accel
        self.v_acts[veh].linear.x = self.vels[veh].linear.x

    def get_lead_vel(self, t):
        vel = next(self.lead_vels)
        while vel[1]['Time'] < t:
            vel = next(self.lead_vels)
        return vel[1]['linear.x']

    def callback3(self, msg):
        self.v_ref = msg
        # v_ref data: time, RL controller output, predicted acceleration
        self.v_ref_data.append([self.t, msg.linear.x, msg.linear.z])

    def v_ref_callback_gen(self, veh):
        # Based on callback3
        def callback(msg):
            self.v_refs[veh] = msg
            # v_ref data: time, RL controller output, predicted acceleration
            self.v_ref_datas[veh].append([self.t, msg.linear.x, msg.linear.z])
        return callback

    def callback2(self, msg):
        self.v_act = msg
        self.sub_data.append([self.t, msg.linear.x])

    def v_act_callback_gen(self, veh):
        print('callback for', veh)
        # Based on callback_2
        def callback(msg):
            print('in callback for', veh)
            self.v_act = msg
            self.sub_datas[veh].append([self.t, msg.linear.x])
        return callback

    def callback(self, msg):
        print('callback being called')
        callback_t = time.time() - self.t0
        if len(self.sub_data) == 0:
            self.vel.linear.x = max(msg.linear.x, 0)
            self.sub_data.append([self.t, max(msg.linear.x, 0)])
        else:
            if callback_t - self.sub_data[-1][0] > 0.035:
                self.vel.linear.x = max(msg.linear.x, 0)
                self.sub_data.append([self.t, max(msg.linear.x, 0)])
            else:
                pass

    def shutdown(self):
        if not os.path.exists(os.path.join(self.node_path, 'data/')):
            os.makedirs(os.path.join(self.node_path, 'data/'))
        save_path = os.path.join(self.node_path, 'data/sumo_log.npy')
        np.save(save_path, self.sumo_data)
        # Change it from dictionary to array
        if self.avs:
            save_path = os.path.join(self.node_path, 'data/pub_msgs.npy')
            len_pubs = min([len(x) for x in self.pub_datas.values()])
            pubs = np.stack([self.pub_datas[veh][:len_pubs] for veh in self.avs])
            np.save(save_path, pubs)
            save_path = os.path.join(self.node_path, 'data/sub_msgs.npy')
            len_subs = min([len(x) for x in self.sub_datas.values()])
            subs = np.stack([self.sub_datas[veh][:len_subs] for veh in self.avs])
            np.save(save_path, subs)
            len_refs = min([len(x) for x in self.v_ref_datas.values()])
            save_path = os.path.join(self.node_path, 'data/v_ref_msgs.npy')
            refs = np.stack([self.v_ref_datas[veh][:len_refs] for veh in self.avs])
            np.save(save_path, refs)
        print('')
        rospy.loginfo('Closing SUMO host node...')
        self.kernel.close()


    def add_vehicles_debug(self):
        start_dist = 50*len(self.platoon_names)
        for name in self.platoon_names:
            self.kernel.vehicle.add(name, 'Eastbound_3', departPos=str(start_dist))
            self.kernel.vehicle.setSpeedMode(name, 0)
            self.kernel.vehicle.setLaneChangeMode(name, 0)
            self.starting_positions.append(start_dist)
            start_dist -= 50
        self.car_len = self.kernel.vehicle.getLength(self.platoon_names[0])


    def get_sim_state(self):
        ahead_edge = self.kernel.vehicle.getRoadID('lead')
        ahead_edgepos = self.kernel.vehicle.getLanePosition('lead')
        ahead_pos = self.kernel.vehicle.getPosition('lead')
        ahead_vel = self.kernel.vehicle.getSpeed('lead')
        for veh in self.platoon_names[1:]:
            dx = self.kernel.vehicle.getDrivingDistance(veh, ahead_edge, ahead_edgepos)
            behind_vel = self.kernel.vehicle.getSpeed(veh)
            behind_edge = self.kernel.vehicle.getRoadID(veh)
            behind_edgepos = self.kernel.vehicle.getLanePosition(veh)
            behind_pos = self.kernel.vehicle.getPosition(veh)

            self.vels[veh].linear.x = behind_vel
            # self.space_gap.data = dx
            self.rel_vels[veh].linear.x = ahead_vel - behind_vel
            dist = np.sqrt((ahead_pos[0]-behind_pos[0])**2 + (ahead_pos[1]-behind_pos[1])**2)
            # Subtract car length and sumo min-gap from distance
            dist -= self.car_len
            dist -= self.min_gap
            if dx < 0:
                print('car ' + veh + ' ahead of its leader')
                dist *= -1
            self.space_gaps[veh].data = dist

            ahead_edge = behind_edge
            ahead_edgepos = behind_edgepos
            ahead_pos = behind_pos
            ahead_vel = behind_vel


    def set_speed_debug(self):
        next_vel = self.v_act.linear
        self.kernel.vehicle.setSpeed('ego', next_vel)
        self.kernel.vehicle.setSpeed('lead', 1)


    def get_data_debug(self):
        t = self.kernel.simulation.getTime() - self.kernel.simulation.getDeltaT()
        data = []
        for veh in self.platoon_names:
            pos = self.kernel.vehicle.getPosition(veh)
            vel = self.kernel.vehicle.getSpeed(veh)
            data.append([t, pos[0], pos[1], vel, self.space_gaps[veh].data, self.rel_vels[veh].linear.x])

        return data



if __name__ == '__main__':
    args = parse_args()
    node = None
    try:
        rospy.init_node('sumo_host_node', anonymous=False)
        node = SumoHostNode(args)
        node.shutdown()

    except traci.exceptions.TraCIException:
        print('\nERROR IN TraCI\n')
        if node:
            node.shutdown()
    except rospy.ROSInterruptException:
        pass

