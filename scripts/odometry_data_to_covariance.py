#! /usr/bin/env python

import numpy as np
import sys
import math
import random
from numpy import dtype
import h5py
import cv
from __builtin__ import min
from odometry_cnn_data import Odometry

np.set_printoptions(linewidth=200)

if len(sys.argv) < 2:
    sys.stderr.write("Expected arguments: <pose-file>+\n")
    sys.exit(1)

poses_6dof = []
for arg in sys.argv[1:]:
    lines = open(arg).readlines()
    prev = Odometry()
    for line in lines:
        kitti_pose = map(float, line.strip().split())
        o = Odometry(kitti_pose)
        poses_6dof.append(np.array((o-prev).dof))
        prev = o

average_pose = sum(poses_6dof)/len(poses_6dof)
poses_6dof_normalized = [pose - average_pose for pose in poses_6dof]
print "average_pose", average_pose

print "euclidean loss: ", sum([np.linalg.norm(pose) for pose in poses_6dof_normalized])/len(poses_6dof_normalized)

covariance = np.zeros((6,6))
for pose in poses_6dof_normalized:
    covariance += np.outer(pose, pose)
covariance *= 1.0/len(poses_6dof_normalized)
print covariance

precision = np.linalg.inv(covariance)
for i in range(6):
    pose = np.copy(poses_6dof_normalized[0])
    pose[i] += 0.1
    print "i", i, np.dot(np.dot(pose, precision), pose)
print "mahalanobis loss: ", sum([math.sqrt(np.dot(np.dot(pose, precision), pose)) for pose in poses_6dof_normalized])/len(poses_6dof_normalized)

std_dev = [math.sqrt(covariance[i, i]) for i in range(6)]
print "std_dev", std_dev
print pose
for i in range(6):
    pose = np.copy(poses_6dof_normalized[0])
    pose[i] += 0.1
    print "i", i, [((pose - average_pose)[i]/std_dev[i]) for i in range(6)]
