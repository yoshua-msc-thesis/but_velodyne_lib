#! /usr/bin/env python

import sys, os
import numpy as np
from transformations import quaternion_matrix

VELO_TO_IMU = np.array([0,0,1,
                        1,0,0,
                        0,1,0]).reshape((3,3))
Y_UP = np.array([-1,0,0,
                 0,-1,0,
                 0,0,1]).reshape((3,3))

if len(sys.argv) != 5:
    sys.stderr.write("ERROR, expecting arguments [qx, qy, qz, qw]\n")
    sys.exit(1)

quat = map(float, [sys.argv[4]] + sys.argv[1:4])
M = np.linalg.inv(quaternion_matrix(quat).transpose())  # OK

M[0:3,0:3] = np.matmul(np.matmul(np.matmul(Y_UP, np.linalg.inv(VELO_TO_IMU)), M[0:3,0:3]), VELO_TO_IMU)

for r in range(3):
    for c in range(4):
        sys.stdout.write("%s "%(M[r,c],))
print ""
