#! /usr/bin/env python

import sys
import matplotlib.pyplot as plt
import math
import numpy as np
import h5py
import numpy
import random
import itertools

from odometry_cnn_data import load_kitti_poses
from odometry_cnn_data import Odometry
from odometry_cnn_data import get_delta_odometry

ROTATIONS_BINS = [20, 100, 20]
MAX_ROTATIONS = [r*180/math.pi for r in [0.025, 0.1, 0.02]]     # deg
BIN_SIZES = [MAX_ROTATIONS[i]/ROTATIONS_BINS[i] for i in range(3)]

def display_histograms(odometries, bins):
    rotations = {}
    rotations["x"] = [o.dof[3] for o in odometries]
    rotations["y"] = [o.dof[4] for o in odometries]
    rotations["z"] = [o.dof[5] for o in odometries]
    
    for key, rots in rotations.iteritems():
        rots.sort()
        rots = rots[0:int(0.999*len(rots))]
        plt.title("Histogram of angles %s"%(key))
        plt.hist(rots, bins=100)  # plt.hist passes it's arguments to np.histogram
        plt.show()
        plt.clf()
#odometries = list(itertools.chain.from_iterable([get_delta_odometry(load_kitti_poses(file)) for file in sys.argv[1:]]))
#display_histograms(odometries, BINS)

def round_angles(odom):
    for i in range(3):
        odom.dof[i+3] = round(odom.dof[i+3]/BIN_SIZES[i])*BIN_SIZES[i]
    odom.setMFromDof()
    return odom

def round_pose_files():
    for file in sys.argv[1:]:
        odoms = map(round_angles, get_delta_odometry(load_kitti_poses(file)))
        with open(file + ".round", "w") as out_file:
            pose = Odometry()
            for o in odoms:
                pose.move(o.dof)
                out_file.write("%s\n"%(pose))

def angles2classes(angles):
    classes = [0]*3
    for i in range(3):
        cls = round(angles[i]/(BIN_SIZES[i]*2))+ROTATIONS_BINS[i]/2
        cls = max(0, min(cls, ROTATIONS_BINS[i]-1))
        classes[i] = cls
    return classes

for filename in sys.argv[1:]:
    with h5py.File(filename, "r") as in_hf:
        with h5py.File(filename + ".rotclasses", "w") as out_hf:
            data = in_hf["data"][:]
            odometries = in_hf["odometry"][:]
            odom_count = np.shape(odometries)[0]
            
            out_hf.create_dataset('data', np.shape(data), dtype='f4')
            out_hf.create_dataset('odometry', np.shape(odometries), dtype='f4')
            out_hf.create_dataset('rot_class_x', (odom_count,), dtype='f4')
            out_hf.create_dataset('rot_class_y', (odom_count,), dtype='f4')
            out_hf.create_dataset('rot_class_z', (odom_count,), dtype='f4')
            
            out_hf['data'][...] = data
            out_hf['odometry'][...] = odometries
    
            for i in range(odom_count):
                out_hf['rot_class_x'][i], out_hf['rot_class_y'][i], out_hf['rot_class_z'][i] = angles2classes(odometries[i][3:])
