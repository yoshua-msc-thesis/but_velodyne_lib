#! /usr/bin/env python

import sys
import argparse
import math

from odometry_cnn_data import Odometry
from kitti_poses_rot_quantization import classes2single_angle 

parser = argparse.ArgumentParser(description="Probabilities to pose file")
parser.add_argument("-i", "--angle_idx", dest="angle_idx", type=int, required=True)
parser.add_argument("-c", "--classes", dest="classes", type=int, required=True)
parser.add_argument("-m", "--max_value", dest="max_value", type=float, required=True)
parser.add_argument("-w", "--window_size", dest="window_size", type=int, required=True)
args = parser.parse_args()

assert 0 <= args.angle_idx < 3

pose = Odometry()
print pose

max_rotations = [0]*3
max_rotations[args.angle_idx] = args.max_value
classes_counts = [1]*3
classes_counts[args.angle_idx] = args.classes

for line in sys.stdin.readlines():
    probabilities = map(float, line.split())
    dof = [0]*6
    angle = classes2single_angle(probabilities, args.angle_idx, max_rotations, classes_counts, args.window_size)
    dof[args.angle_idx+3] = angle * math.pi/180.0
    pose.move(dof)
    print pose
