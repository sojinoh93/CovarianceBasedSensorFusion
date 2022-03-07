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

class KalmanFilter:
    def __init__(self, state_len):
        self.F = np.eye(state_len)
        self.B = np.eye(state_len)
        self.I = np.eye(state_len)
        self.H = np.eye(state_len)

    def estimate(self, x, P, u, Q):
        x_bar = self.F * x + self.B * u       
        P_bar = self.F * P * self.F.T + Q     
        return x_bar, P_bar

    def update(self, x_bar, P_bar, z, R):
        y = z - self.H * x_bar
        S = self.H * P_bar * self.H.T + R
        S_inv = np.linalg.inv(S)
        K = P_bar * self.H.T * S_inv
        x2 = x_bar + K * y
        P2 = (self.I - K * self.H) * P_bar
        return x2, P2

class ExtendedKalmanFilter:
    def __init__(self, state_len):
        self.F = np.matrix([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ])
        self.B = np.eye(state_len)
        self.I = np.eye(state_len)
        self.H = np.eye(state_len)

    def estimate(self, x, P, u, Q):
        x_bar = self.F * x + self.B * u       
        P_bar = self.F * P * self.F.T + Q     
        return x_bar, P_bar

    def update(self, x_bar, P_bar, z, R):
        y = z - self.H * x_bar
        S = self.H * P_bar * self.H.T + R
        S_inv = np.linalg.inv(S)
        K = P_bar * self.H.T * S_inv
        x2 = x_bar + K * y
        P2 = (self.I - K * self.H) * P_bar
        return x2, P2