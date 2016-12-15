#! /usr/bin/env python

import sys
import math
import numpy as np
import h5py
import random

from odometry_cnn_data import load_kitti_poses
from odometry_cnn_data import Odometry
from odometry_cnn_data import get_delta_odometry

BATCH_SIZE = 16
OUT_LAYERS = ['rot_class_x', 'rot_class_y', 'rot_class_z']

class Line:
    def __init__(self, k, q):
        self.k = k
        self.q = q
        
    def y(self, x):
        return self.k*x + self.q

class Duplication:

    BINS = 20

    def __init__(self, angles):
        hist, self.edges = np.histogram(angles, Duplication.BINS)
        line = Line(-hist[0]/self.edges[-1], hist[0])
        self.duplication = []
        for i in range(len(hist)):
            self.duplication.append(line.y(self.edges[i])/hist[i])

    def getDuplication(self, angle):
        i = 0
        while self.edges[i] < angle:
            i += 1
            if i >= len(self.edges):
                print "Skipping angle: %s[rad]"%(angle)
                return 0
        return int(self.duplication[i-1]+0.5)

class NoDuplication:
    def getDuplication(self, angle):
        return 1

def data_index_from_odom(odom_i, hist_size, batch_size, hist_i):
    batch_i = odom_i / batch_size
    slot_i = odom_i % batch_size
    return batch_i*batch_size*hist_size + slot_i + hist_i*batch_size

#odometries = get_delta_odometry(load_kitti_poses(sys.stdin))
#angles = [math.sqrt(sum(math.pow(o.dof[i]*180.0/math.pi, 2.0) for i in range(3,6))) for o in odometries]
#angles.sort()
#angles = angles[0:int(0.999*len(angles))]
#dupl = Duplication(angles)

dupl = NoDuplication()

#===============================================================================
# import matplotlib.pyplot as plt
# dupl_angles = []
# for a in angles:
#     dupl_angles += [a]*dupl.getDuplication(a)
# plt.title("Histogram of angles")
# plt.hist(angles, bins=Duplication.BINS)  # plt.hist passes it's arguments to np.histogram
# plt.hist(dupl_angles, bins=Duplication.BINS)  # plt.hist passes it's arguments to np.histogram
# plt.show()
#===============================================================================

for filename in sys.argv[1:]:
    indices = []
    with h5py.File(filename, "r") as hf:
        data = hf["data"]
        odometry = hf["odometry"]
        odom_count = np.shape(odometry)[0]
        hsize = np.shape(data)[0]/odom_count
        for i in range(odom_count):
            angle = np.linalg.norm(odometry[i][3:6])
            indices += [i]*dupl.getDuplication(angle)
        indices_count = len(indices)/BATCH_SIZE * BATCH_SIZE
        indices = indices[:indices_count]
        random.shuffle(indices)
        output_file = h5py.File(filename + ".shuffled", 'w')
        output_file.create_dataset('data', (len(indices)*hsize,) + np.shape(data)[1:], dtype='f4')
        output_file.create_dataset('odometry', (len(indices),) + np.shape(odometry)[1:], dtype='f4')

        rot_classes = {}
        if OUT_LAYERS[0] in hf:
            for rot_layer in OUT_LAYERS:
                output_file.create_dataset(rot_layer, (len(indices),), dtype='f4')
                rot_classes[rot_layer] = hf[rot_layer][:]

        #print indices, (len(indices)*hsize,) + np.shape(data)[1:]
        for new_i in range(len(indices)):
            output_file['odometry'][new_i] = odometry[indices[new_i]]
            if len(rot_classes) > 0:
                for rot_layer in OUT_LAYERS:
                    output_file[rot_layer][new_i] = rot_classes[rot_layer][indices[new_i]]
            for h in range(hsize):
                from_i = data_index_from_odom(indices[new_i], hsize, BATCH_SIZE, h)
                to_i = data_index_from_odom(new_i, hsize, BATCH_SIZE, h)
                #print from_i, indices[new_i], hsize, BATCH_SIZE, h, odom_count
                #print to_i, new_i, hsize, BATCH_SIZE, h, len(indices), len(indices)*hsize
                output_file['data'][to_i] = data[from_i]
