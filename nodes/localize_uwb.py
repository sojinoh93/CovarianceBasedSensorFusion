#!/usr/bin/env python2.7

# -*- coding: utf-8 -*-
# -*- encoding: utf8 -*-
# -*- decoding: utf-8 -*-

import rospy
from geometry_msgs.msg import Vector3, Twist
from nav_msgs.msg import Odometry
import math
import numpy as np

import filter.linear_kalman_filter as kf
from sensor.global_odom import GlobalOdom

class Initialization:
    def __init__(self, n_sample=30):
        rospy.Subscriber("/robot_pose/uwb", Odometry, self.callback_uwb, n_sample)
        self.n_sample = n_sample

        self.uwb = [0.0, 0.0]
        self.uwb_buf = []

        # Calculate initial pose
        p1 = self.get_sample_mean()
        self.forward()
        p2 = self.get_sample_mean()

        t = math.atan2(p2[1]-p1[1], p2[0]-p1[0])
        print("INFO - initial pose", p1, p2)
        print("INFO - initial pose", p2[0], p2[1], np.degrees(t))

        self.pose = p2
        self.t = t

    def callback_uwb(self, msg, n_sample):
        pose = msg.pose.pose.position
        self.uwb[0] = pose.x
        self.uwb[1] = pose.y
        
        if len(self.uwb_buf) < n_sample:
            self.uwb_buf.append([pose.x, pose.y, pose.z])

    def get_sample_mean(self):
        self.uwb_buf = []

        while (len(self.uwb_buf) < self.n_sample):
            if rospy.is_shutdown():
                break

        mean = np.mean(self.uwb_buf, axis=0)
        return mean

    def forward(self, vx=0.2, t=3):
        pub_move = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.sleep(0.5)
        twist = Twist()
        twist.linear.x = vx
        pub_move.publish(twist)

        rospy.sleep(t)
        twist.linear.x = 0.0
        pub_move.publish(twist)

        rospy.sleep(0.5)

if __name__ == '__main__':
    print("Node: Localize with UWB")
    rospy.init_node('localize_uwb', anonymous=True)
    rate = rospy.Rate(20)
    
    init = Initialization()
    odom = GlobalOdom(init.pose, init.t)
    kalman_filter = kf.KalmanFilter(init.pose)
    pub_kf = rospy.Publisher("/robot_pose/kf", Vector3, queue_size=1000)

    while not rospy.is_shutdown():
        odom.update()
        odom.pub()
        
        x, y = kalman_filter.apply(odom.v, odom.cov, init.uwb)
        msg_kf = Vector3(x, y, 0.0)
        pub_kf.publish(msg_kf)

        rate.sleep()

