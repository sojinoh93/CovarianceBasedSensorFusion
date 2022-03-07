#!/usr/bin/env python2.7

# -*- coding: utf-8 -*-
# -*- encoding: utf8 -*-
# -*- decoding: utf-8 -*-

import rospy
from geometry_msgs.msg import Vector3, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import math
import numpy as np
from math import pi

from filter.kalman_filter import KalmanFilter, ExtendedKalmanFilter
from global_odom import GlobalOdom
from tf_broadcaster import TfBroadcaster

class Initialization:
    def __init__(self, n_sample=30):
        sub_uwb = rospy.Subscriber("/robot_pose/uwb", Odometry, self.callback_uwb, n_sample)
        self.n_sample = n_sample

        self.uwb = [0.0, 0.0]
        self.uwb_buf = []
        self.buf_x = []
        self.buf_y = []

        # Calculate initial pose
        p1 = self.get_sample_mean()

        self.forward()

        # -- Line fitting -- #
        k, m = np.polyfit(self.buf_x, self.buf_y, 1)

        p1_fit = [self.buf_x[0], self.buf_x[0]*k + m] 
        p2_fit = [self.buf_x[-1], self.buf_x[-1]*k + m] 
        
        t_fit = math.atan2(p2_fit[1]-p1_fit[1], p2_fit[0]-p1_fit[0])
        # ------------------ #

        p2 = self.get_sample_mean()
  
        t = math.atan2(p2[1]-p1[1], p2[0]-p1[0])            

        print(np.degrees(t), np.degrees(t_fit))
        print("INFO - initial pose", p2[0], p2[1], np.degrees(t))

        self.pose = p2
        self.t = t # radian

        sub_uwb.unregister()

    def callback_uwb(self, msg, n_sample):
        pose = msg.pose.pose.position
        self.uwb[0] = pose.x
        self.uwb[1] = pose.y
        
        if len(self.uwb_buf) < n_sample:
            self.uwb_buf.append([pose.x, pose.y, pose.z])
        else:
            self.uwb_buf.pop(0)
            self.uwb_buf.append([pose.x, pose.y, pose.z])

        if len(self.buf_x) < 50:
            self.buf_x.append(pose.x)
            self.buf_y.append(pose.y)
        else:
            self.buf_x.pop(0)
            self.buf_y.pop(0)
            self.buf_x.append(pose.x)
            self.buf_y.append(pose.y)

    def get_sample_mean(self):
        self.uwb_buf = []

        while (len(self.uwb_buf) < self.n_sample):
            if rospy.is_shutdown():
                break

        mean = np.mean(self.uwb_buf, axis=0)
        return mean

    def forward(self, vx=0.2, t=5):
        pub_move = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.sleep(0.5)
        twist = Twist()
        twist.linear.x = vx
        pub_move.publish(twist)

        rospy.sleep(t)
        twist.linear.x = 0.0
        pub_move.publish(twist)

        rospy.sleep(0.5)


class Localization:
    def __init__(self, init_pose, init_t):

        self.br = TfBroadcaster()
        self.uwb_kf = KalmanFilter(2)
        self.amcl_kf = ExtendedKalmanFilter(3)
        self.amcl_covariance_is_high = False

        self.x = np.matrix([
            [init_pose[0]],
            [init_pose[1]],
            [init_t]
        ])
        self.P = np.diag([0.1, 0.1, 0.1])
        self.buf_pose = []
        
        self.t = 0.0
        self.t_threshold = 10.0

        rospy.Subscriber("/robot_pose/uwb", Odometry, self.callback_uwb)
        # rospy.Subscriber("/robot_pose/optitrack", Vector3, self.callback_opti)
        ### Turn on TF broadcast
        rospy.Subscriber("/odom", Odometry, self.callback_odom)
        ### Turn on AMCL
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback_amcl)

        self.pub_kf = rospy.Publisher("/robot_pose/kf", Vector3, queue_size=1000)
        self.pub_initialpose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1000)
        self.pub_initial_pose(init_pose[0], init_pose[1], init_t)

        self.pub_amcl_cov = rospy.Publisher("/robot_pose/amcl_cov", Vector3, queue_size=1000)

    def pub_initial_pose(self, x, y, t):
        q = quaternion_from_euler(0.0, 0.0, t)
        msg = PoseWithCovarianceStamped()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.pose.covariance = [
            0.1, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.01, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        rospy.sleep(0.5)
        self.pub_initialpose.publish(msg)
        print("send initial_pose", np.degrees(t))

    def estimate(self, odom_v, odom_cov):
        t = self.x[2][0]
        dist = np.sqrt(odom_v[0] ** 2 + odom_v[1] ** 2)

        dx = dist * math.cos(t)
        dy = dist * math.sin(t)
        dt = np.radians(odom_v[2])

        u = np.matrix([
            [dx],
            [dy],
            [dt]
        ])

        Q = np.diag([odom_cov[0], odom_cov[1], odom_cov[2]]) # State covariance

        self.x, self.P = self.amcl_kf.estimate(self.x, self.P, u, Q)
    
        t = self.x[2][0]
        if t > pi:
            t = t - 2*pi
        elif t < -pi:
            t = 2*pi + t

        self.x[2][0] = t 

        msg_kf = Vector3(self.x.item(0), self.x.item(1), self.x.item(2))
        self.pub_kf.publish(msg_kf)

    def callback_opti(self, msg):
        pass

    def callback_odom(self, msg):
        time = msg.header.stamp
        msg_kf = Vector3(self.x.item(0), self.x.item(1), self.x.item(2))
        self.br.send(msg_kf, time)

    def callback_uwb(self, msg):
        pose = msg.pose.pose.position

        z = np.matrix([
            [pose.x],
            [pose.y]
        ])

        self.buf_pose = [pose.x, pose.y, pose.z]

        dist_z = abs(self.buf_pose[2] - 1.18)
        # dist_z = 0.001
        # dist_z = 0.01
        # dist_z = 0.1

        x = np.matrix([
            [self.x.item(0)],
            [self.x.item(1)]
        ])

        P = np.diag([self.P.item(0), self.P.item(1)])

        # Todo
        R = np.diag([dist_z, dist_z])
        # R = np.diag([0.01, 0.01])

        x, P = self.uwb_kf.update(x, P, z, R)

        self.x[0][0] = x[0][0]
        self.x[1][0] = x[1][0]
        self.P[0][0] = P[0][0]
        self.P[1][0] = P[1][0]

    def callback_amcl(self, msg):
        if len(self.buf_pose) > 0:
            pose = msg.pose.pose.position
            q = msg.pose.pose.orientation
            angle = euler_from_quaternion([q.x, q.y, q.z, q.w]) # radians
            t = angle[2]
            cov_x = msg.pose.covariance[0]
            cov_y = msg.pose.covariance[7]
            cov_t = msg.pose.covariance[35]

            dist_kf_x = (self.buf_pose[0] - pose.x)
            dist_kf_y = (self.buf_pose[1] - pose.y)
            dist_cov = cov_x + cov_y
            dist_z = abs(self.buf_pose[2] - 1.18)

            msg_cov = Vector3(cov_x, dist_z, dist_z ** 2)
            self.pub_amcl_cov.publish(msg_cov)

            if dist_kf_x < cov_x or dist_kf_y < cov_y:
                # Initiate AMCL pose when its covariance is high
                self.pub_initial_pose(self.buf_pose[0], self.buf_pose[1], self.x.item(2))
                self.amcl_covariance_is_high = False

            else:
                z = np.matrix([
                    [pose.x],
                    [pose.y],
                    [t]
                ])
                R = np.diag([cov_x, cov_y, cov_t])
                self.x, self.P = self.amcl_kf.update(self.x, self.P, z, R)
                

if __name__ == '__main__':
    print("Node: Localize with UWB")
    rospy.init_node('localize_uwb', anonymous=True)
    rate = rospy.Rate(10)
    
    init = Initialization()
    odom = GlobalOdom(init.pose, init.t)    
    localize = Localization(init.pose, init.t)

    while True and not rospy.is_shutdown():
        odom.update()
        odom.pub()
        # print("dt", odom.v[2])

        localize.estimate(odom.encoder_v, odom.cov)
        # if(odom.v[0] > 0  or odom.v[1] > 0):
        #     localize.update_t()
        rate.sleep()