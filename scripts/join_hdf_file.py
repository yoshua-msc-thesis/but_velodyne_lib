#! /usr/bin/env python

import sys
import numpy as np
import h5py
import os

BATCH_SIZE=4
MAX_SIZE = 2*1000*1000*1000 # 2GB
OUT_LAYERS = ['rot_class_x', 'rot_class_y', 'rot_class_z']

filenames = []
i = 1
while i < len(sys.argv):
    if sys.argv[i] == "-o":
        out_filename = sys.argv[i+1]
        i += 2
    else:
        filenames.append(sys.argv[i])
        i += 1

if len(out_filename) == 0 or len(filenames) == 0:
    sys.stderr.write("Usage: %s -o <joined-output.hdf5> <input.hdf5>+\n", sys.argv[0])
    sys.exit(1)

total_odom_count = 0
for fn in filenames:
    with h5py.File(fn, "r") as hf:
        odometry = hf["odometry"][:]
        total_odom_count += np.shape(odometry)[0]
        try:
            hsize
        except NameError:
            data = hf["data"][:]
            hsize = np.shape(data)[0]/np.shape(odometry)[0]

output_file = h5py.File(out_filename, 'w')
output_file.create_dataset('data', (total_odom_count*hsize,) + np.shape(data)[1:], dtype='f4')
output_file.create_dataset('odometry', (total_odom_count,) + np.shape(odometry)[1:], dtype='f4')
for rot_layer in OUT_LAYERS:
    output_file.create_dataset(rot_layer, (total_odom_count,), dtype='f4')

odoms_copied = 0
for fn in filenames:
    print "Adding", fn
    with h5py.File(fn, "r") as hf:
        data = hf["data"][:]
        odometry = hf["odometry"][:]
        odom_count = np.shape(odometry)[0]

        rot_classes = {}
        for rot_layer in OUT_LAYERS:
            rot_classes[rot_layer] = hf[rot_layer][:]

        output_file['odometry'][odoms_copied:odoms_copied+odom_count] = odometry[...]
        output_file['data'][odoms_copied*hsize:(odoms_copied+odom_count)*hsize] = data[...]
        for rot_layer in OUT_LAYERS:
            output_file[rot_layer][odoms_copied:odoms_copied+odom_count] = rot_classes[rot_layer][...]

        odoms_copied += odom_count
