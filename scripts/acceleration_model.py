#!/usr/bin/env python
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import numpy as np
import os
import rospy
import time

class AccelerationModelNode:
    def __init__(self):
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, callback=self.callback_cmd_vel)
        self.vel_sub = rospy.Subscriber('/vel', Twist, callback=self.callback_vel)
        self.v_ref_sub = rospy.Subscriber('/v_ref', Twist, callback=self.callback_v_ref)
        self.v_act_pub = rospy.Publisher('/v_act', Twist, queue_size=0)
        self.v_ref = Twist()
        self.vel = Twist()
        self.cmd_vel = Twist()
        self.predicted_vel = Twist()

        # Initialize all variables to 0
        self.p = 0
        self.q = 0
        self.v = 0
        self.a = 0
        self.last_t = 0

        self.t0 = time.time()

        # Stepsize based on frequency at which SumoHostNode publishes velocity
        self.step = 1/20.0

        # rospy.spin()
        # cmd_vels = np.linspace(0, 10, 1000)
        cmd_vels = [10]*1000
        self.test_accel(cmd_vels, 40.0)

    def calculate_predicted_vel(self):
        # Constants for acceleration model
        k1 = 1.745
        k2 = 1.566
        kp = 2.720
        ki = 0.0656
        kd = 1.340
        kn = 7.549

        v_obs = self.vel.linear.x

        # predicted_accel = self.v_ref.linear.z
        v_des = self.cmd_vel.linear.x

        # Run forward Euler equations
        p_next = self.p + self.step*self.q
        q_next = self.q + self.step*(-kn*self.q - self.p + v_des)
        a_next = self.a + self.step*(k1*ki*kn*self.p + k1*ki-k1*kn*kn*kd*self.q \
                -(k1*kd*kn+k1*kp)*self.v - k2*self.a + (k1*kd*kn+k1*kp)*v_des)
        self.predicted_vel.linear.x = v_obs + self.step*self.a

        # Update current variables for next timestep
        self.p = p_next
        self.q = q_next
        self.v = self.predicted_vel.linear.x
        self.a = a_next


    def callback_cmd_vel(self, msg):
        self.cmd_vel = msg

    def callback_v_ref(self, msg):
        self.v_ref = msg

    def callback_vel(self, msg):
        # If vehicle is stopped, reset p and q
        if msg.linear.x == 0:
            self.p = 0
            self.q = 0
            self.v = 0
            self.a = 0

        # Note how fast the vehicle is currently accelerating
        curr_t = time.time() - self.t0
        t_delta = curr_t - self.last_t
        curr_vel = msg.linear.x
        # Do we want to keep track of this?
        # self.accel_obs = (curr_vel - self.vel.linear.x)/t_delta
        self.vel = msg

        # Update step for however long the time difference was
        self.step = t_delta
        self.last_t = curr_t

        # If we calculate the next velocity every time we receive the cmd_vel
        self.calculate_predicted_vel()
        self.v_act_pub.publish(self.predicted_vel)

    def shutdown(self):
        pass

    def test_accel(self, cmd_vels, frequency):
        self.p = 0
        self.q = 0
        self.v = 0
        self.a = 0
        self.last_t = 0
        self.vel.linear.x = 0
        self.t0 = time.time()
        vels = []

        print('starting test')
        for i in cmd_vels:
            self.cmd_vel.linear.x = i
            self.calculate_predicted_vel()
            self.vel = self.predicted_vel
            vels.append((time.time()-self.t0, self.cmd_vel.linear.x, self.vel.linear.x))
            time.sleep(1/frequency)
        print('done testing')
        np.save('accel_test.npy', vels)

if __name__ == '__main__':
    try:
        rospy.init_node('acceleration_model_node', anonymous=False)
        node = AccelerationModelNode()
        node.shutdown()
    except rospy.ROSInterruptException:
        pass

