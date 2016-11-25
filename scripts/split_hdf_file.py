#! /usr/bin/env python

import sys
import numpy as np
import h5py
import os

BATCH_SIZE=4
MAX_SIZE = 2*1000*1000*1000 # 2GB
OUT_LAYERS = ['rot_class_x', 'rot_class_y', 'rot_class_z']

for filename in sys.argv[1:]:
    split_times = os.stat(filename).st_size/(MAX_SIZE)
    with h5py.File(filename, "r") as hf:
        data = hf["data"][:]
        odometry = hf["odometry"][:]
        odom_count = np.shape(odometry)[0]
        hsize = np.shape(data)[0]/odom_count

        rot_classes = {}
        if OUT_LAYERS[0] in hf: 
            for rot_layer in OUT_LAYERS:
                rot_classes[rot_layer] = hf[rot_layer][:]

        batches_count=odom_count/BATCH_SIZE
        if odom_count%BATCH_SIZE > 0:
            print "Warning: file", filename, "not batch aligned. Batch size:", BATCH_SIZE, "odom frames:", odom_count
        while batches_count%split_times != 0:
            split_times += 1
        print filename, "will be split", split_times, "times"

        max_new_odoms_count = odom_count / (split_times+1) + odom_count % (split_times+1)
        odoms_copied = 0
        for s in range(split_times+1):
            new_odoms_count = min(max_new_odoms_count, odom_count-odoms_copied)
            new_filename = filename + ".%s"%(s)
            print new_filename
            output_file = h5py.File(new_filename, 'w')
            output_file.create_dataset('data', (new_odoms_count*hsize,) + np.shape(data)[1:], dtype='f4')
            output_file.create_dataset('odometry', (new_odoms_count,) + np.shape(odometry)[1:], dtype='f4')
            output_file['odometry'][...] = odometry[odoms_copied:odoms_copied+new_odoms_count]
            output_file['data'][...] = data[odoms_copied*hsize:(odoms_copied+new_odoms_count)*hsize]
            if len(rot_classes) > 0:
                for rot_layer in OUT_LAYERS:
                    output_file.create_dataset(rot_layer, (new_odoms_count,), dtype='f4')
                    output_file[rot_layer][...] = rot_classes[rot_layer][odoms_copied:odoms_copied+new_odoms_count]

            odoms_copied += new_odoms_count
