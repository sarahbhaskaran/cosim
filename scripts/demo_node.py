#!/usr/bin/env python
from geometry_msgs.msg import Twist
import numpy as np
import os
import rospy
import traci
import time
from utils import add_vehicles_debug, get_next_vel, get_data_debug


class SumoHostNode:
    def __init__(self):
        sumo_bin = os.path.expanduser('~/GitHub/sumo/bin/sumo')
        self.node_path = os.path.dirname(os.path.realpath(__file__))
        sumo_cfg = os.path.join(self.node_path, 'ring/cfg/ring.sumo.cfg')
        sumo_cmd = [sumo_bin,
                    '-c', sumo_cfg,
                    '--seed', '42',
                    '--step-length', '0.05',
                    '--step-method.ballistic']
        traci.start(sumo_cmd, label='sim')
        self.kernel = traci.getConnection('sim')
        add_vehicles_debug(self.kernel)
        self.kernel.simulationStep()

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=0)
        self.sub = rospy.Subscriber('/vel', Twist, self.callback)
        self.pub_data = []
        self.sub_data = []
        self.cmd_vel = Twist()
        self.vel = Twist()

        self.sumo_data = []
        self.t0 = time.time()
        self.rate = rospy.Rate(20)
        self.t = time.time() - self.t0
        self.pub.publish(self.cmd_vel)
        self.pub_data.append([self.t, self.cmd_vel.linear.x])
        for t in range(1000):
            self.rate.sleep()
            self.sumo_data.append(get_data_debug(self.kernel, 'fs'))
            self.kernel.vehicle.setSpeed('lead', 0)
            self.kernel.vehicle.setSpeed('ego', self.vel.linear.x)
            self.kernel.simulationStep()
            self.t = time.time() - self.t0
            if t >= 1 / 0.05:
                self.cmd_vel.linear.x = get_next_vel(self.kernel, 'fs')
            self.pub.publish(self.cmd_vel)
            self.pub_data.append([self.t, self.cmd_vel.linear.x])

    def callback(self, msg):
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
        print('')
        rospy.loginfo('Closing SUMO host node...')
        self.kernel.close()


if __name__ == '__main__':
    try:
        rospy.init_node('sumo_host_node', anonymous=False)
        node = SumoHostNode()
        node.shutdown()
    except rospy.ROSInterruptException:
        pass
