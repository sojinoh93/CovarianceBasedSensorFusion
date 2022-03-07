#!/usr/bin/env python2.7

# -*- coding: utf-8 -*-
# -*- encoding: utf8 -*-
# -*- decoding: utf-8 -*-

import rospy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import math
from math import pi
import numpy as np

import lineaer_kalman_filter
import extended_kalman_filter
import unscented_kalman_filter

class TestKalmanFilter:
    def __init__(self):
        sub_uwb = rospy.Subscriber("/uwb", Odometry, self.uwb_callback)
        sub_odom = rospy.Subscriber("/odom_global", Odometry, self.odom_callback)
        self.pub_kf = rospy.Publisher("/kf", Vector3, queue_size=10)
        
        self.kf_initialized = False
        self.x = np.matrix([
            [0.0],
            [0.0]
        ])
        self.P = np.diag([0.1, 0.1])
        self.u = np.matrix([
            [0.0],
            [0.0]
        ])
        self.z = np.array([
            [0.0],
            [0.0]
        ])

        self.Q = np.diag([0.0001, 0.0001]) # State covariances
        self.R = np.diag([0.1, 0.1]) # Observation covariance

    def uwb_callback(self, msg):
        self.z[0][0] = msg.pose.pose.position.x
        self.z[1][0] = msg.pose.pose.position.y

    def odom_callback(self, msg):
        if not self.kf_initialized:
            initial_pose = msg.pose.pose.position
            self.x[0][0] = initial_pose.x
            self.x[1][0] = initial_pose.y
            self.kf_initialized = True

        twist = msg.twist.twist 
        self.u[0][0] = twist.linear.x
        self.u[1][0] = twist.linear.y
        
        self.x, self.P = lineaer_kalman_filter.predict(self.x, self.P, self.u, self.z, self.Q, self.R)

        kf_pose = Vector3(self.x.item(0), self.x.item(1), 0.0)
        self.pub_kf.publish(kf_pose)

if __name__ == '__main__':
    print("Node: Test Kalman filter")
    rospy.init_node('test_kalman_filter', anonymous=True)
    test = TestKalmanFilter()
    rospy.spin()
