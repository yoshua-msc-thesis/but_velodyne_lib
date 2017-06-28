#! /usr/bin/env python

import sys
from odometry_cnn_data import load_kitti_poses, Odometry

poses = load_kitti_poses(sys.stdin)

# to ROS
axis_correction = Odometry([
        0, -1,  0,  0,
        0,  0, -1,  0,
        1,  0,  0,  0 ])

sys.stderr.write("%s\n"%(axis_correction.dof))

for p in poses:
    p = axis_correction.inv() * p * axis_correction
    print ("%.4f "*6)%(p.dof[0], p.dof[1], p.dof[2], p.dof[3], p.dof[4], p.dof[5])
