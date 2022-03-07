#!/usr/bin/env python2.7

# -*- coding: utf-8 -*-
# -*- encoding: utf8 -*-
# -*- decoding: utf-8 -*-


from genpy.message import _get_message_or_service_class
import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from silbot3_msgs.msg import Device_WheelEncoder_Msg

import math
from math import pi
import numpy as np
from tf.msg import tfMessage

class GlobalOdom:
    def __init__(self, pose_init=[0.0, 0.0], t_init=0.0):
        # Encoder
        self.encoder_initiated = False
        self.encoder = [0.0, 0.0, 0.0]
        self.encoder_v = [0.0, 0.0, 0.0]

        # Topic msg
        self.msg_odom = Odometry()
        self.msg_angle = Float32()
        self.msg_tf = TransformStamped()

        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.callback_odom)
        self.sub_encoder = rospy.Subscriber("/DeviceNode/WheelEncoder/data", Device_WheelEncoder_Msg, self.callback_encoder)
        self.pub_odom_global = rospy.Publisher("/robot_pose/odom_global", Odometry, queue_size=10)
        self.pub_angle = rospy.Publisher("/robot_pose/angle", Float32, queue_size=10)
        self.pub_encoder = rospy.Publisher("/robot_pose/v", Vector3, queue_size=10)

        # Global odom
        self.x = 0.0
        self.y = 0.0
        self.t = 0.0
        self.q = Quaternion()

        self.v = [0.0, 0.0, 0.0]
        self.v_min = 0.0001
        
        self.cov = [0.0, 0.0, 0.1**2]
        self.cov_pose = 0.0005

        # Local odom
        self.__odom_x = 0.0
        self.__odom_y = 0.0
        self.__odom_t = 0.0

        # Update
        self.initialized = False
        self.sn = 1e-20  # large number
        self.ln = 1e+7  # large number
        self.x_offset = 0.0
        self.y_offset = 0.0
        self.T = [pose_init[0], pose_init[1]]
        self.t_offset = t_init

            
    def initial_pose(self, pose_init, t_init):
        self.x = pose_init[0]
        self.y = pose_init[1]
        self.T = [pose_init[0], pose_init[1]]
        self.t_offset = t_init

    def callback_odom(self, msg):
        q = msg.pose.pose.orientation
        angle = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.__odom_x = msg.pose.pose.position.x
        self.__odom_y = msg.pose.pose.position.y
        self.__odom_t = angle[2]

    def callback_encoder(self, msg):
        if not self.encoder_initiated:
            self.encoder = [msg.x, msg.y, msg.t]
            self.encoder_initiated = True
        else:
            self.encoder_v[0] = (msg.x - self.encoder[0]) / 1000.0
            self.encoder_v[1] = (msg.y - self.encoder[1]) / 1000.0
            self.encoder_v[2] = msg.t - self.encoder[2]
            self.encoder = [msg.x, msg.y, msg.t]

            v = Vector3(self.encoder_v[0], self.encoder_v[1], self.encoder_v[2])
            self.pub_encoder.publish(v)

    def update(self):
        x = self.__odom_x
        y = self.__odom_y
        t = self.__odom_t

        if not self.initialized:
            self.x_offset = x
            self.y_offset = y
            self.t_offset -= t 
            self.initialized = True

        """ 1) Update t """
        t = t + self.t_offset
        if t > pi:
            t = t - 2*pi
        elif t < -pi:
            t = 2*pi + t

        t_global = np.degrees(t)
        self.q = quaternion_from_euler(0.0, 0.0, t)
    
        """ 2) Update pose """
        x_nor = x - self.x_offset
        y_nor = y - self.y_offset

        a = math.cos(self.t_offset)
        b = math.sin(self.t_offset)
        R = [a, -b, b, a]

        x_global = R[0] * x_nor + R[1] * y_nor + self.T[0]
        y_global = R[2] * x_nor + R[3] * y_nor + self.T[1]

        self.v[0] = x_global - self.x
        self.v[1] = y_global - self.y
        self.v[2] = t_global - self.t

        """ Exception for no moving """ 
        self.cov[0] = self.sn if abs(self.v[0]) < self.v_min else self.cov_pose
        self.cov[1] = self.sn if abs(self.v[1]) < self.v_min else self.cov_pose

        # """ Exception for emergency mode """
        # if self.v[0] == 0 and self.v[1] == 0 and self.v[2] != 0:
        #     self.cov[0] = self.ln
        #     self.cov[1] = self.ln

        self.x = x_global
        self.y = y_global
        self.t = t_global
    
    def pub(self):
        self.msg_odom.header.stamp = rospy.Time.now()  
        self.msg_odom.pose.pose.position.x = self.x
        self.msg_odom.pose.pose.position.y = self.y
        self.msg_odom.pose.pose.orientation.x = self.q[0]
        self.msg_odom.pose.pose.orientation.y = self.q[1]
        self.msg_odom.pose.pose.orientation.z = self.q[2]
        self.msg_odom.pose.pose.orientation.w = self.q[3]
        self.msg_odom.pose.covariance = [self.cov[0], 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, self.cov[1], 0.0, 0.0, 0.0, 0.0,  
                                        0.0, 0.0, self.ln, 0.0, 0.0, 0.0,  
                                        0.0, 0.0, 0.0, self.ln, 0.0, 0.0,  
                                        0.0, 0.0, 0.0, 0.0, self.ln, 0.0,  
                                        0.0, 0.0, 0.0, 0.0, 0.0, self.cov[2]]
        self.msg_odom.twist.twist.linear.x = self.v[0]
        self.msg_odom.twist.twist.linear.y = self.v[1]
        self.msg_odom.twist.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub_odom_global.publish(self.msg_odom)

        self.msg_angle = self.t
        self.pub_angle.publish(self.msg_angle)