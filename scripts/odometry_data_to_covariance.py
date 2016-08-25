#! /usr/bin/env python

import numpy as np
import sys
import math
import random
from numpy import dtype
import h5py
import cv
from __builtin__ import min
from eulerangles import mat2eulerZYX

np.set_printoptions(linewidth=200)

class Odometry:
    def __init__(self, kitti_pose = [1, 0, 0, 0, 
                                     0, 1, 0, 0, 
                                     0, 0, 1, 0]):
        assert len(kitti_pose) == 12
        self.dof = [0]*6
        self.M = np.matrix([[0]*4, [0]*4, [0]*4, [0, 0, 0, 1]], dtype=np.float64)
        for i in range(12):
            self.M[i/4, i%4] = kitti_pose[i]
        self.setDofFromM()
    
    def setDofFromM(self):
        R = self.M[:3, :3]
        self.dof[0], self.dof[1], self.dof[2] = self.M[0, 3], self.M[1, 3], self.M[2, 3]
        self.dof[5], self.dof[4], self.dof[3] = mat2eulerZYX(R)
  
    def distanceTo(self, other):
        sq_dist = 0
        for i in range(3):
            sq_dist += (self.dof[i]-other.dof[i])**2
        return math.sqrt(sq_dist)

    def __mul__(self, other):
        out = Odometry()
        out.M = self.M * other.M
        out.setDofFromM()
        return out
    
    def __sub__(self, other):
        out = Odometry()
        out.M = np.linalg.inv(other.M) * self.M
        out.setDofFromM()
        return out
    
if len(sys.argv) < 2:
    sys.stderr.write("Expected arguments: <pose-file>+\n")
    sys.exit(1)

poses_6dof = []
lines = open(sys.argv[1]).readlines()
prev = Odometry()
for line in lines:
    kitti_pose = map(float, line.strip().split())
    o = Odometry(kitti_pose)
    poses_6dof.append(np.array((o-prev).dof))
    prev = o

average_pose = sum(poses_6dof)/len(lines)
poses_6dof_normalized = [pose - average_pose for pose in poses_6dof]
print average_pose

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
print std_dev
print pose
for i in range(6):
    pose = np.copy(poses_6dof_normalized[0])
    pose[i] += 0.1
    print "i", i, [((pose - average_pose)[i]/std_dev[i]) for i in range(6)]
