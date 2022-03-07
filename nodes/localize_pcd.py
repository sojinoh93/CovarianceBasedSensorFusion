#!/usr/bin/env python2.7

# -*- coding: utf-8 -*-
# -*- encoding: utf8 -*-
# -*- decoding: utf-8 -*-

import rospy
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Empty, String
import math
import numpy as np
from gist_pub_msgs.msg import gist_robot_pos_msgs

import filter.linear_kalman_filter as kf
from sensor.global_odom import GlobalOdom

class Initialization:
    def __init__(self, n_sample=10):
        # Silbot <--> Kinect 
        self.sub_pcd = rospy.Subscriber("/gist_robot_pos_msg", gist_robot_pos_msgs, self.callback_pcd, n_sample)  
        self.pub_init_pose_signal = rospy.Publisher('/gist_tracking/robot_init_pose_signal', String, queue_size=10)

        # Silbot <--> Task manager 
        self.sub_on_place_signal = rospy.Subscriber("/gist_tracking/robot_on_place", Empty, self.callback_on_place)
        self.pub_start_signal = rospy.Publisher('/gist_tracking/robot_tracking_start', Empty, queue_size=100)

        self.ok = False
        self.pcd = [0.0, 0.0]
        self.n_sample = n_sample
        self.pose = [0.0, 0.0]
        self.pose_buf = []
        self.t = 0.0

    def callback_on_place(self, msg):
        self.pub_init_pose_signal.publish(String("start"))
        p1 = self.get_sample_mean()

        self.forward()

        self.pub_init_pose_signal.publish(String("stop"))
        p2 = self.get_sample_mean()
        
        t = math.atan2(p2[1]-p1[1], p2[0]-p1[0])
        print("INFO - initial pose", p2[0], p2[1], np.degrees(t))

        self.pose = p2
        self.t = np.pi / 2.0#t
        self.pub_start_signal.publish(Empty())
        self.ok = True

    def callback_pcd(self, msg, n_sample):
        self.pcd[0] = msg.x
        self.pcd[1] = msg.y
        
        if len(self.pose_buf) < n_sample:
            self.pose_buf.append([msg.x, msg.y])

    def get_sample_mean(self):
        self.pose_buf = []

        while (len(self.pose_buf) < self.n_sample):
            if rospy.is_shutdown():
                break

        mean = np.mean(self.pose_buf, axis=0)
        return mean

    def forward(self, vx=0.2, t=3):
        vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.sleep(0.5)
        twist = Twist()
        twist.linear.x = vx
        vel_pub.publish(twist)

        rospy.sleep(t)
        twist.linear.x = 0.0
        vel_pub.publish(twist)

        rospy.sleep(0.5)

if __name__ == '__main__':
    print("Node: Localize with PCD")
    rospy.init_node('localize_pcd', anonymous=True)
    rate = rospy.Rate(100)

    init = Initialization()
    odom = GlobalOdom()
    kalman_filter = kf.KalmanFilter()
    pub_kf = rospy.Publisher("/gist_tracking/robot_pose_filtered", gist_robot_pos_msgs, queue_size=1000)

    while not rospy.is_shutdown():
        if init.ok:
            odom.initial_pose(init.pose, init.t)
            kalman_filter.initial_state(init.pose)
            break

    while not rospy.is_shutdown():
        odom.update()
        odom.pub()

        x, y = kalman_filter.apply(odom.v, odom.cov, init.pcd)
        msg_kf = gist_robot_pos_msgs(0, x, y, 500.0)
        pub_kf.publish(msg_kf)

        rate.sleep()

