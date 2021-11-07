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


class SumoHostNode:
    def __init__(self):
        sumo_bin = os.environ['SUMO_HOME']
        self.node_path = os.path.dirname(os.path.realpath(__file__))
        sumo_cfg = os.path.join(self.node_path, 'I24/I24.sumo.cfg')
        sumo_cmd = [sumo_bin,
                    '-c', sumo_cfg,
                    '--seed', '42',
                    '--step-length', '0.05',
                    '--step-method.ballistic']
        traci.start(sumo_cmd, label='sim')
        self.kernel = traci.getConnection('sim')
        self.add_vehicles_debug()
        self.kernel.simulationStep()

        # self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=0)
        # self.sub = rospy.Subscriber('/vel', Twist, self.callback)
        self.ego_vel_pub = rospy.Publisher('/vel', Twist, queue_size=0)
        self.space_gap_pub = rospy.Publisher('/lead_dist', Float64, queue_size=0)
        self.rel_vel_pub = rospy.Publisher('/rel_vel', Twist, queue_size=0)
        self.acc_pub = rospy.Publisher('/msg_467', Point, queue_size=0)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, callback=self.callback2)
        self.v_ref_sub = rospy.Subscriber('/v_ref', Twist, callback=self.callback3)
        self.pub_data = []
        self.sub_data = []
        self.v_ref_data = []
        self.vel = Twist()
        self.rel_vel = Twist()
        self.space_gap = Float64()
        self.acc = Point()
        self.acc.y = 10
        self.cmd_vel = Twist()
        self.v_ref = Twist()

        # Make lead vel have the velocity of a real car
        lead_vel_csv = os.path.join(self.node_path, 'car_vel.csv')
        self.lead_vels = pd.read_csv(lead_vel_csv).iterrows()

        self.sumo_data = []
        self.t0 = time.time()
        self.rate = rospy.Rate(20)
        self.t = time.time() - self.t0
        # self.pub.publish(self.cmd_vel)
        self.pub_data.append([self.t, self.vel.linear.x])
        #for t in range(4320):
        for t in range(2000):
            self.rate.sleep()
            self.sumo_data.append(self.get_data_debug())

            curr_lead_vel = self.get_lead_vel(t)


            self.kernel.vehicle.setSpeed('lead', curr_lead_vel)
            self.kernel.vehicle.setSpeed('ego', self.cmd_vel.linear.x)

            self.kernel.simulationStep()

            self.t = time.time() - self.t0
            # if t >= 1 / 0.05:
            #     self.cmd_vel.linear.x = get_next_vel(self.kernel, 'fs')
            # self.pub.publish(self.cmd_vel)

            # Get info about current stuff
            self.get_sim_state()
            # Publish info that RL controller takes as input
            self.ego_vel_pub.publish(self.vel)
            self.space_gap_pub.publish(self.space_gap)
            self.rel_vel_pub.publish(self.rel_vel)
            self.acc_pub.publish(self.acc)


            self.pub_data.append([self.t, self.vel.linear.x])

    def get_lead_vel(self, t):
        vel = next(self.lead_vels)
        while vel[1]['Time'] < t:
            vel = next(self.lead_vels)
        return vel[1]['linear.x']

    def callback3(self, msg):
        self.v_ref = msg
        # v_ref data: time, RL controller output, predicted acceleration
        self.v_ref_data.append([self.t, msg.linear.x, msg.linear.z])

    def callback2(self, msg):
        self.cmd_vel = msg
        self.sub_data.append([self.t, msg.linear.x])

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
        np.save(save_path, self.pub_data)
        save_path = os.path.join(self.node_path, 'data/sub_msgs.npy')
        np.save(save_path, self.sub_data)
        save_path = os.path.join(self.node_path, 'data/v_ref_msgs.npy')
        np.save(save_path, self.v_ref_data)
        print('')
        rospy.loginfo('Closing SUMO host node...')
        self.kernel.close()


    def add_vehicles_debug(self):
        self.kernel.vehicle.add('ego', 'Eastbound_3', departPos='0')
        self.kernel.vehicle.setSpeedMode('ego', 0)
        self.kernel.vehicle.add('lead', 'Eastbound_3', departPos='50')
        self.kernel.vehicle.setSpeedMode('lead', 0)


    def get_sim_state(self):
        lead_edge = self.kernel.vehicle.getRoadID('lead')
        lead_edgepos = self.kernel.vehicle.getLanePosition('lead')
        dx = self.kernel.vehicle.getDrivingDistance('ego', lead_edge, lead_edgepos)
        dx = dx - self.kernel.vehicle.getLength('lead')
        dx = dx - self.kernel.vehicle.getMinGap('ego')
        vego = self.kernel.vehicle.getSpeed('ego')
        vlead = self.kernel.vehicle.getSpeed('lead')
        self.vel.linear.x = vego
        self.space_gap.data = dx
        self.rel_vel.linear.x = vlead - vego
        # self.accel?

    def set_speed_debug(self):
        next_vel = self.cmd_vel.linear
        self.kernel.vehicle.setSpeed('ego', next_vel)
        self.kernel.vehicle.setSpeed('lead', 1)


    def get_data_debug(self):
        t = self.kernel.simulation.getTime() - self.kernel.simulation.getDeltaT()
        ego_pos = self.kernel.vehicle.getDistance('ego')
        ego_vel = self.kernel.vehicle.getSpeed('ego')
        # TODO make ego_acc correct
        ego_acc = 0
        # Add 50 because the lead vehicle started 50 ahead of the ego vehicle
        lead_pos = self.kernel.vehicle.getDistance('lead') + 50
        lead_vel = self.kernel.vehicle.getSpeed('lead')
        lead_acc = 0
        return [t, ego_pos, ego_vel, ego_acc, lead_pos, lead_vel, lead_acc]


if __name__ == '__main__':
    try:
        rospy.init_node('sumo_host_node', anonymous=False)
        node = SumoHostNode()
        node.shutdown()
    except rospy.ROSInterruptException:
        pass

