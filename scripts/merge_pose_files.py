#! /usr/bin/env python

import sys
from odometry_cnn_data import load_kitti_poses
from odometry_cnn_data import get_delta_odometry
from odometry_cnn_data import Odometry

if len(sys.argv) != 3:
    sys.stderr.write("Invalid arguments, expected: <translation.poses> <rotation.poses>")
    sys.exit(1)

t_poses = load_kitti_poses(sys.argv[1])
r_poses = load_kitti_poses(sys.argv[2])

t_odoms = get_delta_odometry(t_poses, [1]*len(t_poses))
r_odoms = get_delta_odometry(r_poses, [1]*len(r_poses))

assert len(t_odoms) == len(r_odoms)

pose = Odometry()
for i in range(len(t_odoms)):
    dof = t_odoms[i].dof[0:3] + r_odoms[i].dof[3:6]
    pose.move(dof)
    print pose
    sys.stderr.write("%s\n" % r_odoms[i].dof[3:6])
