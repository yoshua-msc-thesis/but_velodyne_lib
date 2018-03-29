#! /usr/bin/env python

import sys, math
import autograd.numpy as np
from autograd import grad

from transformations import matrix_to_axis_angle
from odometry_cnn_data import Odometry, load_kitti_poses

POSES1 = []
POSITIONS2 = []
SIGMA = 0.05
LEARNING_RATE = 0.0002


def dof2mat(dof):
    A = np.cos(dof[5]); B = np.sin(dof[5])
    C = np.cos(dof[4]); D = np.sin(dof[4])
    E = np.cos(dof[3]); F = np.sin(dof[3])
    DE = D*E; DF = D*F;

    return np.array([A * C, A * DF - B * E, B * F + A * DE, dof[0],
                     B * C, A * E + B * DF, B * DE - A * F, dof[1],
                     -D,    C * F,          C * E,          dof[2],
                     0.0,   0.0,            0.0,            1.0]).reshape(4, 4)


def t_and_axis_angle_to_matrix(t_and_axis_angles):
    c = np.cos(t_and_axis_angles[6])
    s = np.sin(t_and_axis_angles[6])
    t = 1 - c
    x, y, z = t_and_axis_angles[3:6]
    return np.array([t*x*x + c,	  t*x*y - z*s,	t*x*z + y*s, t_and_axis_angles[0],
                     t*x*y + z*s, t*y*y + c,    t*y*z - x*s, t_and_axis_angles[1],
                     t*x*z - y*s, t*y*z + x*s,  t*z*z + c,   t_and_axis_angles[2],
                     0.0,         0.0,          0.0,         1.0]).reshape(4,4)

def loss(calib_params_xyzaa):
    C = t_and_axis_angle_to_matrix(calib_params_xyzaa)
    l = 0
    for P,t in zip(POSES1, POSITIONS2):
        #l += np.linalg.norm((C*P*np.linalg.inv(C))[0:3, 3] - t)
        delta = np.linalg.norm((C*P*np.linalg.inv(C))[0:3, 3] - t)
        l += 1.0 - np.exp(-1/(2*SIGMA) * delta)
    return l


# 6DOF [0.02925498, 0.02516794, 0.24983137, -1.87076251, -0.0765768, -3.10943351] ZYX order
# mat: [-0.996553883963  -0.0825491077691  0.00812411007599 0.0292549802962
#       -0.0320593740478  0.292985139492  -0.955579355455   0.0251679367657
#        0.0765019796715 -0.952546771998  -0.294621951428   0.249831374382 ]
# AA { [ -0.0356556, 0.8039481, -0.5936296 ], 3.1841315 }
#   or [ -0.113532, 2.5598766, -1.8901947 ]

if len(sys.argv) != 3:
    sys.stderr.write("ERROR, expected arguments: <calibration-data> <init-6dof-calibration>\n")
    sys.exit(0)

calibration = load_kitti_poses(sys.argv[2])[0]
aa = matrix_to_axis_angle(calibration.M)
calibration_xyzaa = np.concatenate([calibration.dof[0:3], aa[0], np.array([aa[1]])])

for line in open(sys.argv[1]).readlines():
    data = map(float, line.split())
    P = np.asarray(data[0:12] + [0.0, 0.0, 0.0, 1.0]).reshape((4, 4))
    t = np.asarray(data[12:])
    POSES1.append(P)
    POSITIONS2.append(t)

gradient = grad(loss)

learning_rate = LEARNING_RATE
for i in range(10000):
    if i % 500 == 0:
        print('Iteration %-4d | Loss: %.4f' % (i, loss(calibration_xyzaa)))
        learning_rate /= 2
        print t_and_axis_angle_to_matrix(calibration_xyzaa)
    g = gradient(calibration_xyzaa)
    calibration_xyzaa -= g * learning_rate
    print calibration_xyzaa

print t_and_axis_angle_to_matrix(calibration_xyzaa)
