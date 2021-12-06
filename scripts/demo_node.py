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
        self.platoon_names = [veh + str(i) for i, veh in enumerate(self.platoon_set)]
        self.platoon_names[0] = 'lead'
        self.avs = [i for i, veh in enumerate(self.platoon_names) if 'av' in veh]
        sumo_cmd = [sumo_bin,
                    '-c', sumo_cfg,
                    '--seed', '42',
                    '--step-length', '0.05',
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

        self.ego_vel_pubs = {i: rospy.Publisher('av'+str(i)+'/vel', Twist, queue_size=0) for i in self.avs}
        self.space_gap_pubs = {i: rospy.Publisher('av'+str(i)+'/lead_dist', Float64, queue_size=0) for i in self.avs}
        self.rel_vel_pubs = {i: rospy.Publisher('av'+str(i)+'/rel_vel', Twist, queue_size=0) for i in self.avs}
        self.acc_pubs = {i: rospy.Publisher('av'+str(i)+'/msg_467', Point, queue_size=0) for i in self.avs}
        self.v_act_subs = {i: rospy.Subscriber('av'+str(i)+'/v_act', Twist, callback=self.v_act_callback_gen(i)) for i in self.avs}
        self.v_ref_subs = {i: rospy.Subscriber('av'+str(i)+'/v_ref', Twist, callback=self.v_ref_callback_gen(i)) for i in self.avs}

        self.pub_datas = [[]]*len(self.platoon_names)
        self.sub_datas = [[]]*len(self.platoon_names)
        self.v_ref_datas = [[]]*len(self.platoon_names)
        self.vels = {i: Twist() for i in self.avs}
        self.rel_vels = {i: Twist() for i in self.avs}
        self.space_gaps = {i: Float64() for i in self.avs}
        self.accs = {i: Point() for i in self.avs}
        for acc in self.accs.values():
            acc.y = 100
        self.v_acts = {i: Twist() for i in self.avs}
        self.v_refs = {i: Twist() for i in self.avs}

        # Make lead vel have the velocity of a real car
        lead_vel_csv = os.path.join(self.node_path, 'car_vel.csv')
        self.lead_vels = pd.read_csv(lead_vel_csv).iterrows()

        self.sumo_data = []
        self.t0 = time.time()
        self.rate = rospy.Rate(20)
        self.t = time.time() - self.t0
        for i in self.avs:
            self.pub_datas[i].append([self.t, self.vels[i].linear.x])
        # for t in range(4320):
        for t in range(1000):
            self.rate.sleep()
            self.sumo_data.append(self.get_data_debug())

            curr_lead_vel = self.get_lead_vel(t)
            # curr_lead_vel = 15

            self.kernel.vehicle.setSpeed('lead', curr_lead_vel)
            for i in range(len(self.platoon_names)):
                if 'av' in self.platoon_names[i]:
                    self.kernel.vehicle.setSpeed(self.platoon_names[i], self.v_acts[i].linear.x)
                else:
                    pass

            self.kernel.simulationStep()

            self.t = time.time() - self.t0

            # Get info about current stuff
            self.get_sim_state()
            # Publish info that RL controller takes as input
            for i in self.avs:
                self.ego_vel_pubs[i].publish(self.vels[i])
                self.space_gap_pubs[i].publish(self.space_gaps[i])
                self.rel_vel_pubs[i].publish(self.rel_vels[i])
                self.acc_pubs[i].publish(self.accs[i])

                self.pub_datas[i].append([self.t, self.vels[i].linear.x])


    def get_lead_vel(self, t):
        vel = next(self.lead_vels)
        while vel[1]['Time'] < t:
            vel = next(self.lead_vels)
        return vel[1]['linear.x']

    def callback3(self, msg):
        self.v_ref = msg
        # v_ref data: time, RL controller output, predicted acceleration
        self.v_ref_data.append([self.t, msg.linear.x, msg.linear.z])

    def v_ref_callback_gen(self, i):
        # Based on callback3
        def callback(msg):
            self.v_refs[i] = msg
            # v_ref data: time, RL controller output, predicted acceleration
            self.v_ref_datas[i].append([self.t, msg.linear.x, msg.linear.z])
        return callback

    def callback2(self, msg):
        self.v_act = msg
        self.sub_data.append([self.t, msg.linear.x])

    def v_act_callback_gen(self, i):
        # Based on callback_2
        def callback(msg):
            self.v_act = msg
            self.sub_data.append([self.t, msg.linear.x])
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
        save_path = os.path.join(self.node_path, 'data/pub_msgs.npy')
        np.save(save_path, self.pub_datas)
        save_path = os.path.join(self.node_path, 'data/sub_msgs.npy')
        np.save(save_path, self.sub_datas)
        save_path = os.path.join(self.node_path, 'data/v_ref_msgs.npy')
        np.save(save_path, self.v_ref_datas)
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
        for i in range(1, len(self.platoon_names)):
            dx = self.kernel.vehicle.getDrivingDistance(self.platoon_names[i], ahead_edge, ahead_edgepos)
            behind_vel = self.kernel.vehicle.getSpeed(self.platoon_names[i])
            behind_edge = self.kernel.vehicle.getRoadID(self.platoon_names[i])
            behind_edgepos = self.kernel.vehicle.getLanePosition(self.platoon_names[i])
            behind_pos = self.kernel.vehicle.getPosition(self.platoon_names[i])
            if i in self.avs:
                self.vels[i].linear.x = behind_vel
                # self.space_gap.data = dx
                self.rel_vels[i].linear.x = ahead_vel - behind_vel
                dist = np.sqrt((ahead_pos[0]-behind_pos[0])**2 + (ahead_pos[1]-behind_pos[1])**2)
                # Subtract car length and sumo min-gap from distance
                dist -= self.car_len
                dist -= self.min_gap
                if dx < 0:
                    print('car ' + self.platoon_names[i] + ' ahead of its leader')
                    dist *= -1
                self.space_gaps[i].data = dist

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
        for i in range(len(self.platoon_names)):
            pos = self.kernel.vehicle.getPosition(self.platoon_names[i])
            vel = self.kernel.vehicle.getSpeed(self.platoon_names[i])
            if 'av' in self.platoon_names[i]:
                data.append([t, pos[0], pos[1], vel, self.space_gaps[i].data, self.rel_vels[i].linear.x])
            else:
                # No space gaps for non rl vehicles for now
                data.append([t, pos[0], pos[1], vel, 0, 0])

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

