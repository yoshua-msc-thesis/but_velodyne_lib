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
from odometry_cnn_data import gen_preserve_mask
from odometry_cnn_data import mask_list

# for nomal data
ROTATIONS_BINS = [20, 100, 20]
MAX_ROTATIONS = [r * 180 / math.pi for r in [0.025, 0.1, 0.02]]  # deg

# for skipped frames
# ROTATIONS_BINS = [20, 200, 20]
# MAX_ROTATIONS = [r*180/math.pi for r in [0.02, 0.2, 0.02]]     # to deg

BIN_SIZES = [MAX_ROTATIONS[i] / ROTATIONS_BINS[i] for i in range(3)]

OUT_LAYERS = ['rot_class_x', 'rot_class_y', 'rot_class_z']

def display_histograms(odometries, bins):
    rotations = {}
    rotations["x"] = [abs(o.dof[3]) for o in odometries]
    rotations["y"] = [abs(o.dof[4]) for o in odometries]
    rotations["z"] = [abs(o.dof[5]) for o in odometries]
    
    for key, rots in rotations.iteritems():
        rots.sort()
        rots = rots[0:int(0.99 * len(rots))]
        plt.title("Histogram of angles %s" % (key))
        plt.hist(rots, bins=100)  # plt.hist passes it's arguments to np.histogram
        plt.show()
        plt.clf()

def round_angles(odom):
    for i in range(3):
        odom.dof[i + 3] = round(odom.dof[i + 3] / BIN_SIZES[i]) * BIN_SIZES[i]
    odom.setMFromDof()
    return odom

def round_pose_files():
    for file in sys.argv[1:]:
        odoms = map(round_angles, get_delta_odometry(load_kitti_poses(file)))
        with open(file + ".round", "w") as out_file:
            pose = Odometry()
            for o in odoms:
                pose.move(o.dof)
                out_file.write("%s\n" % (pose))

def angles2classes(angles):
    classes = [0] * 3
    for i in range(3):
        cls = round(angles[i] / (BIN_SIZES[i] * 2)) + ROTATIONS_BINS[i] / 2
        cls = max(0, min(cls, ROTATIONS_BINS[i] - 1))
        classes[i] = cls
    return classes

def class_i2angle(dimension_idx, class_idx):
    return (class_idx - ROTATIONS_BINS[dimension_idx] / 2) * (BIN_SIZES[dimension_idx] * 2)

def classes_blob2angles(predicion_blob, blob_i):
    rotations = [0] * 3
    for dim_i in range(3):
        classes = predicion_blob[OUT_LAYERS[dim_i]][blob_i]
        angles_sum = 0
        prob_sum = 0
        for cls_i in range(ROTATIONS_BINS[dim_i]):
            prob = classes[cls_i]
            angles_sum += class_i2angle(dim_i, cls_i) * prob
            prob_sum += prob
        rotations[dim_i] = angles_sum / prob_sum
    return rotations

if __name__ == "__main__":

#     odometries = []
#     for file in sys.argv[1:]:
#         poses = load_kitti_poses(file)
#         for prob in [0, 0.8]:
#             mask = gen_preserve_mask(poses, prob, 60*3.6)
#             odometries += get_delta_odometry(mask_list(poses, mask))
#     display_histograms(odometries, 100)
#     sys.exit(0)

    for filename in sys.argv[1:]:
        with h5py.File(filename, "r") as in_hf:
            with h5py.File(filename + ".rotclasses", "w") as out_hf:
                data = in_hf["data"][:]
                odometries = in_hf["odometry"][:]
                odom_count = np.shape(odometries)[0]

                out_hf.create_dataset('data', np.shape(data), dtype='f4')
                out_hf.create_dataset('odometry', np.shape(odometries), dtype='f4')
                out_hf.create_dataset(OUT_LAYERS[0], (odom_count,), dtype='f4')
                out_hf.create_dataset(OUT_LAYERS[1], (odom_count,), dtype='f4')
                out_hf.create_dataset(OUT_LAYERS[2], (odom_count,), dtype='f4')

                out_hf['data'][...] = data
                out_hf['odometry'][...] = odometries

                for i in range(odom_count):
                    out_hf[OUT_LAYERS[0]][i], out_hf[OUT_LAYERS[1]][i], out_hf[OUT_LAYERS[2]][i] = angles2classes(odometries[i][3:])
