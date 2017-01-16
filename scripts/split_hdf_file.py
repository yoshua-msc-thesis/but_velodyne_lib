#! /usr/bin/env python

import sys
import numpy as np
import h5py
import os

BATCH_SIZE=1
MAX_SIZE = 1800*1000*1000 # 2GB
OUT_LAYERS = ['rot_class_x', 'rot_class_y', 'rot_class_z']

filenames = []
fi = 1
while fi < len(sys.argv):
    if sys.argv[fi] == "-o":
        out_filename = sys.argv[fi+1]
        fi += 2
    else:
        filenames.append(sys.argv[fi])
        fi += 1

if len(out_filename) == 0 or len(filenames) == 0:
    sys.stderr.write("Usage: %s -o <joined-output.hdf5> <input.hdf5>+\n", sys.argv[0])
    sys.exit(1)

for filename in filenames:
    split_times = os.stat(filename).st_size/(MAX_SIZE)
    with h5py.File(filename, "r") as hf:
        data = hf["data"]
        odometry = hf["odometry"]
        odom_count = np.shape(odometry)[0]
        hsize = np.shape(data)[0]/odom_count

        rot_classes = {}
        if OUT_LAYERS[0] in hf: 
            for rot_layer in OUT_LAYERS:
                rot_classes[rot_layer] = hf[rot_layer]

        print filename, "will be split", split_times-1, "times"

        max_new_odoms_count = odom_count / (split_times+1) + odom_count % (split_times+1)
        odoms_copied = 0
        for s in range(split_times):
            new_odoms_count = min(max_new_odoms_count, odom_count-odoms_copied)
            new_filename = out_filename + ".%s"%(s)
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
