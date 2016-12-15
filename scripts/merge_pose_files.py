#! /usr/bin/env python

import sys
import argparse

from odometry_cnn_data import load_kitti_poses
from odometry_cnn_data import get_delta_odometry
from odometry_cnn_data import Odometry

parser = argparse.ArgumentParser(description="Merging multiple pose files")
parser.add_argument("-t", "--translation", dest="translation", type=str, required=False)
parser.add_argument("-r", "--rotation", dest="rotation", type=str, required=False)
parser.add_argument("-rx", "--rx", dest="rx", type=str, required=False)
parser.add_argument("-ry", "--ry", dest="ry", type=str, required=False)
parser.add_argument("-rz", "--rz", dest="rz", type=str, required=False)
args = parser.parse_args()

dof_subsets = ["translation", "rotation",
               "rx", "ry", "rz"]
dof_span = {"translation":[0, 1, 2],
            "rotation":[3, 4, 5],
            "rx":[3],
            "ry":[4],
            "rz":[5]}

dof_odoms = {}
odoms_count = 0
for dof in dof_subsets:
    filename = getattr(args, dof)
    if filename is not None:
        sys.stderr.write("Loading %s\n"%(filename)) 
        poses = load_kitti_poses(filename)
        dof_odoms[dof] = get_delta_odometry(poses)
        if odoms_count == 0:
            odoms_count = len(dof_odoms[dof])
        assert odoms_count == len(dof_odoms[dof])

pose = Odometry()
for i in range(odoms_count):
    total_dof = [0]*6
    for dof_key in dof_subsets:
        if dof_key in dof_odoms:
            for index in dof_span[dof_key]:
                total_dof[index] = dof_odoms[dof_key][i].dof[index]
    pose.move(total_dof)
    print pose
