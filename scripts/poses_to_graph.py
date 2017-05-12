#! /usr/bin/env python

import sys
from odometry_cnn_data import Odometry, Edge3D, load_kitti_poses, get_delta_odometry

odoms = get_delta_odometry(load_kitti_poses(sys.stdin))
for i in range(1, len(odoms)):
    print Edge3D(i-1, i, odoms[i].dof)
