#! /usr/bin/env python

import sys, os
import tempfile
from transformations import quaternion_matrix
import numpy as np
from odometry_cnn_data import Odometry


class ImuReading:
    def __init__(self, data):
        self.duration = data[0]
        acceleration = np.asarray(data[1:4])
        quaternion = [data[7]] + data[4:7]
        self.R = quaternion_matrix(quaternion)[0:3, 0:3].transpose()
        self.calibrated_acc = np.dot(np.linalg.inv(self.R), acceleration)


class PointCloud:
    def __init__(self):
        self.points = np.asarray([])

    def __init__(self, points):
        self.points = points

    def append(self, point):
        self.points.append(point, axes=0)

    def save(self, filename):
        points_count, _ = self.points.shape
        fd = open(filename, "w")
        fd.write("""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH %s
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS %s
DATA ascii
""" % (points_count, points_count))
        for pi in range(points_count):
            fd.write(("%s " * 3 + "\n") % tuple(np.nditer(self.points[pi, 0:3])))

    def show(self):
        _, fn = tempfile.mkstemp(suffix=".pcd")
        self.save(fn)
        os.system("pcl_viewer -ax 1 " + fn)
        os.system("rm " + fn)


readings = []
gravity = np.zeros(3)
for line in sys.stdin.readlines():
    data = map(float, line.split())
    reading = ImuReading(data)
    readings.append(reading)
    gravity += reading.calibrated_acc
gravity /= len(readings)

print gravity, np.linalg.norm(gravity)

positions = np.zeros((len(readings)+1, 3))
velocity = np.zeros(3)
for i,r in enumerate(readings):
    velocity += (r.calibrated_acc - gravity)*r.duration
    positions[i+1, :] = positions[i, :] + velocity*r.duration

PointCloud(positions).show()
for t,r in zip(positions, readings):
    pose = Odometry()
    pose.M[0:3, 0:3] = r.R
    pose.M[0:3, 3] = t.reshape((3,1))
    pose.setDofFromM()
    print pose
