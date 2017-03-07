#! /usr/bin/env python

import sys
import math
import numpy as np
import h5py
import random

from odometry_cnn_data import load_kitti_poses
from odometry_cnn_data import Odometry
from numpy import inf

max_allowed_distance = 1

if len(sys.argv) > 1:
    max_allowed_distance = int(sys.argv[1])

poses = load_kitti_poses(sys.stdin)

for (i1, pose1) in enumerate(poses[:int(len(poses)*0.1)]):
    min_i2 = -1
    min_distance = inf
    for i2 in range(int(len(poses)*0.9), len(poses)):
        dist = pose1.distanceTo(poses[i2])
        if dist < min_distance and dist < max_allowed_distance:
            min_distance = dist
            min_i2 = i2
    if min_i2 >= 0:
        print i1, min_i2, min_distance
