#!/usr/bin/env python2.7

# -*- coding: utf-8 -*-
# -*- encoding: utf8 -*-
# -*- decoding: utf-8 -*-
from logging import disable
import rospy
from std_msgs.msg import String, Float32
from silbot3_msgs.msg import Device_Wheel_Msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion
import numpy as np
from math import pi
import math

class BasicMove(object):
    def __init__(self):
        self.pub_move = rospy.Publisher('/DeviceNode/Wheel/commands', Device_Wheel_Msg, queue_size=1)
        self.pub_state = rospy.Publisher("/robot_pose/state", String, queue_size=100)
        
        self.msg_move = Device_Wheel_Msg()
        self.msg_state = String("stop")

    def pub(self, vx, vy, vt):
        rospy.sleep(0.5)
        self.msg_move.command = "WHEEL_MOVE_BY_VELOCITY_XYT"
        self.msg_move.dParams = [vx, vy, vt]
        self.pub_move.publish(self.msg_move)

    def stop(self):
        self.pub(0.0, 0.0, 0.0)
        self.msg_state = "stop"
        self.pub_state.publish(self.msg_state)

    def forward(self, vx):
        self.pub(vx, 0.0, 0.0)
        self.msg_state = "forward"
        self.pub_state.publish(self.msg_state)

    def backward(self, vx):
        self.pub(-vx, 0.0, 0.0)
        self.msg_state = "backward"
        self.pub_state.publish(self.msg_state)

    def turn(self, vt):
        self.pub(0.0, 0.0, vt)
        self.msg_state = "turn"
        self.pub_state.publish(self.msg_state)

    def distance_angle(self, t1, t2):
        d1 = abs(t1 - t2)
        d2 = 360-d1
        direction = 1 if d1 < d2 else -1
        return min(d1, d2), direction

    def distance(self, p1, p2):
        dist = np.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)
        return abs(dist)
        
    def target_angle(self, p1, p2):
        t = math.atan2(p2.y - p1.y, p2.x - p1.x)
        return t

class Navigation(object):
    def __init__(self, vx=200, vt=20):
        self.move = BasicMove()
        rospy.Subscriber("/robot_pose/kf", Vector3, self.callback_pose)
        rospy.Subscriber("/robot_pose/angle", Float32, self.callback_angle)
        
        self.vx = vx
        self.tx = vt

        self.pose_threshold = 0.2    # m
        self.angle_threshold = 10.0  # degree
        
        self.pose = Vector3()
        self.t = 0.0
        self.initial_pose = Vector3()

    def callback_pose(self, msg):
        self.pose = msg

    def callback_angle(self, msg):
        self.t = msg.data

    def turn_to_goal(self, goal):
        t_goal = self.move.target_angle(self.pose, goal)
        dist_angle, direction = self.move.distance_angle(self.t, t_goal)
        
        print(dist_angle, t_goal, self.t)
        if dist_angle < self.angle_threshold:
            self.move.stop()
            return True
        else:
            self.move.turn(self.tx * direction)
            return False

    def forward_to_goal(self, goal):
        if self.move.distance(self.pose, goal) < self.pose_threshold:
            self.move.stop()
            return True            
        else:
            self.move.forward(self.vx)
            return False
    
    def go_to_goal(self, goal, do_turn=False):
        if self.turn_to_goal(goal):
            if self.forward_to_goal(goal):
                if do_turn:
                    return self.turn_to_goal(self.initial_pose)
                return True
            else:
                return False
        else:
            self.initial_pose = self.pose
            return False

def callback_goal(msg, nav):
    rate = rospy.Rate(20)
    goal = msg
    while nav.go_to_goal(goal):
        rate.sleep()
        

if __name__ == '__main__':
    print("Node: Move")
    rospy.init_node('move', anonymous=True)

    nav = Navigation()
    rospy.sleep(1.0)
    rospy.Subscriber("/robot_pose/goal", Vector3, callback_goal, nav)
    rospy.spin()
    
