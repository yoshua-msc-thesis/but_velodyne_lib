#! /usr/bin/env python

import argparse
from odometry_cnn_data import Odometry, Edge3D, load_kitti_poses

parser = argparse.ArgumentParser(description="Registered frames pairs to EDGE3D")
parser.add_argument("--poses", dest="poses", type=str, required=True)
parser.add_argument("--corrections", dest="corrections", type=str, required=True)
parser.add_argument("--cumulated_frames", dest="cumulated_frames", type=int, required=True)
args = parser.parse_args()

poses = load_kitti_poses(args.poses)

for line in open(args.corrections).readlines():
    tokens = line.split()
    src_i, trg_i = map(int, tokens[0:2])
    pose_elements = map(float, tokens[2:])
    correction = Odometry(pose_elements)
    for i in range(args.cumulated_frames):
        src_pose = poses[src_i+i]
        trg_pose = correction * poses[trg_i+i]
        t = src_pose.inv() * trg_pose
        print Edge3D(src_i+i, trg_i+i, t.dof)
