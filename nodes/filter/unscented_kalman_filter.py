#!/usr/bin/env python2.7

# -*- coding: utf-8 -*-
# -*- encoding: utf8 -*-
# -*- decoding: utf-8 -*-

from math import pi
import numpy as np

# State model matrice.
F = np.matrix([[1, 0], [0, 1]])
H = np.matrix([[1, 0], [0, 1]])
B = np.matrix([[1, 0], [0, 1]])
I = np.eye(len(F))

def smoothing(x, P, u, z, Q, R):
    # 1. Predict
    x_bar = F * x + B * u       # x' = F*x + B*u
    P_bar = F * P * F.T + Q     # P' = F*Pt*Ft + Q

    # 2. Update
    y = z - H * x_bar           # y = z - H*x'
    S = H * P_bar * H.T + R     # S = H*P'*Ht + R
    S_inv = np.linalg.inv(S)    # S_inv = inv(S)
    K = P_bar * H.T * S_inv     # K = P'*Ht*S_inv
    x2 = x_bar + K * y          # x = x' + K*y
    P2 = (I - K * H) * P_bar    # P = (I - K*H)*P'

    return x2, P2

def predict(x_bar, P_bar, u, z, Q, R):
    # 1. Update
    y = z - H * x_bar           # y = z - H*x'
    S = H * P_bar * H.T + R     # S = H*P'*Ht + R
    S_inv = np.linalg.inv(S)    # S_inv = inv(S)
    K = P_bar * H.T * S_inv     # K = P'*Ht*S_inv
    x = x_bar + K * y           # x = x' + K*y
    P = (I - K * H) * P_bar     # P = (I - K*H)*P'

    # 2. Predict
    x_bar2 = F * x + B * u      # x' = F*x + B*u
    P_bar2 = F * P * F.T + Q    # P' = F*Pt*Ft + Q

    return x_bar2, P_bar2

class KalmanFilter:
    def __init__(self, pose_init=[0.0, 0.0]):
        self.x = np.matrix([
            [pose_init[0]],
            [pose_init[1]]
        ])
        self.P = np.diag([0.1, 0.1])
        self.Q = np.diag([0.0001, 0.0001]) # State covariance
        self.R = np.diag([0.1, 0.1]) # Observation covariance

    def initial_state(self, pose_init):
        self.x = np.matrix([
            [pose_init[0]],
            [pose_init[1]]
        ])

    def apply(self, odom_v, odom_cov, uwb):
        u = np.matrix([
            [odom_v[0]],
            [odom_v[1]]
        ])
        
        z = np.matrix([
            [uwb[0]],
            [uwb[1]]
        ])

        self.Q[0][0] = odom_cov[0]
        self.Q[1][1] = odom_cov[1]

        self.x, self.P = predict(self.x, self.P, u, z, self.Q, self.R)

        return self.x.item(0), self.x.item(1)
