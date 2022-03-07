#!/usr/bin/env python2.7

# -*- coding: utf-8 -*-
# -*- encoding: utf8 -*-
# -*- decoding: utf-8 -*-

from math import pi
import numpy as np
import math

""" Modeling
i) x, y: Linear model --> Cannot be extended
[x'] = [1 0][x] + [1 0][dx] 
[y']   [0 1][y]   [0 1][dy]

     = [x + dx]
       [y + dy]

     = [1 0][x] + [dt 0][vx] 
       [0 1][y]   [0 dt][vy]

     = [1 0][x] + [dt*sina 0][v] 
       [0 1][y]   [0 dt*cosa]
     
     = [x] + [d*sina] 
       [y]   [d*cosa]

     = [x + d*sina] 
       [y + d*cosa]

ii) x, y, a
[x'] = [1 0 0][x] + [dt*sin(a+b) 0][v] 
[y']   [0 1 0][y]   [dt*cos(a+b) 0][va]
[a']   [0 0 1][a]   [0          dt] 

[x'] = [1 0 0][x] + [d*sin(a+b)]
[y']   [0 1 0][y]   [d*cos(a+b)]
[a']   [0 0 1][a]   [     b    ]
     
     = [x] + [d*sin(a+b)]
       [y]   [d*cos(a+b)]
       [a]   [     b    ]

     = [x + d*sin(a+b)]
       [y + d*cos(a+b)]
       [a +     b     ]
"""

# State model matrice.
F = np.matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
B = np.matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
I = np.eye(len(F))
H = np.matrix([[1, 0, 0], [0, 1, 0], [0, 0, 0]])


def jacobian(H):
    return H

def estimate(x, P, u, Q):
    # 1. Predict
    x_bar = F * x + B * u       # x' = F*x + B*u
    P_bar = F * P * F.T + Q     # P' = F*Pt*Ft + Q
    return x_bar, P_bar

def update(x_bar, P_bar, z, R):
    # 2. Update
    y = z - H * x_bar           # y = z - H*x'
    S = H * P_bar * H.T + R     # S = H*P'*Ht + R
    S_inv = np.linalg.inv(S)    # S_inv = inv(S)
    K = P_bar * H.T * S_inv     # K = P'*Ht*S_inv
    x2 = x_bar + K * y          # x = x' + K*y
    P2 = (I - K * H) * P_bar    # P = (I - K*H)*P'

    return x2, P2

class ExtendedKalmanFilter:
    def __init__(self, init_pose=[0.0, 0.0], init_t=0.0):
        self.x = np.matrix([
            [init_pose[0]],
            [init_pose[1]],
            [init_t]
        ])
        self.P = np.diag([0.1, 0.1, 0.1])
        self.Q = np.diag([0.0001, 0.0001, 0.0001]) # State covariance
        # self.R = np.diag([0.1, 0.1, 0.1]) # Observation covariance

    def initial_state(self, init_pose, init_t):
        self.x = np.matrix([
            [init_pose[0]],
            [init_pose[1]],
            [0]
        ])

    def estimate(self, odom_v, odom_cov):
        dist = np.sqrt(odom_v[0] ** 2 + odom_v[1] ** 2)
        t = self.x.item(2)
        
        dx = dist * math.cos(t)
        dy = dist * math.sin(t)
        dt = np.radians(odom_v[2])

        u = np.matrix([
            [dx],
            [dy],
            [dt]
        ])

        self.Q[0][0] = odom_cov[0]
        self.Q[1][1] = odom_cov[1]
        self.Q[2][2] = odom_cov[2]

        self.x, self.P = estimate(self.x, self.P, u, self.Q)

        t = self.x[2][0]
        if t > pi:
            t = t - 2*pi
        elif t < -pi:
            t = 2*pi + t

        self.x[2][0] = t 

        return self.x.item(0), self.x.item(1), self.x.item(2)

    def update(self, pose, cov):
        z = np.matrix([
            [pose[0]],
            [pose[1]],
            [0]
        ])

        # Todo
        R = cov

        self.x, self.P = update(self.x, self.P, z, H, R)

        t = self.x[2][0]
        if t > pi:
            t = t - 2*pi
        elif t < -pi:
            t = 2*pi + t

        self.x[2][0] = t

        return self.x.item(0), self.x.item(1), self.x.item(2)

    def update_t(self, t):
        H = np.matrix([[0, 0, 0], [0, 0, 0], [0, 0, 1]])
        z = np.matrix([
            [0],
            [0],
            [t]
        ])

        self.x, self.P = update(self.x, self.P, z, H, self.R)

        return self.x.item(0), self.x.item(1), self.x.item(2)
