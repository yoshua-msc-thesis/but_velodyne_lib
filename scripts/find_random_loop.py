#! /usr/bin/env python

import sys
import math
import numpy as np
import random

from odometry_cnn_data import load_kitti_poses
from odometry_cnn_data import get_delta_odometry
from odometry_cnn_data import Odometry
from numpy import inf

max_allowed_distance = 1

if len(sys.argv) > 1:
    max_allowed_distance = int(sys.argv[1])

poses = load_kitti_poses(sys.stdin)
trajectories = [0]
deltas = get_delta_odometry(poses)
for i in range(1, len(deltas)):
    trajectories.append(trajectories[-1] + deltas[i].distanceTo(deltas[i-1]))

trg_i = -1
min_distance = inf
for _ in range(len(poses)):                                                # attempts
    src_i = random.randint(0, len(poses)-1)
    src_pose = poses[src_i]

    for i in range(len(poses)):
        dist = poses[src_i].distanceTo(poses[i])
        trajectory_diff = trajectories[src_i] - trajectories[i]
        if dist < min_distance and dist < max_allowed_distance and i != src_i and 10*dist < trajectory_diff:
            trg_i = i
            min_distance = dist

    if trg_i >= 0:
        diff = poses[trg_i] - poses[src_i]
        print src_i, trg_i
        print Odometry()
        print str(diff)
        break
