#! /usr/bin/env python

import sys
import numpy as np
import h5py
import os

MAX_SIZE = 2*1000*1000*1000 # 2GB

for filename in sys.argv[1:]:
    split_times = os.stat(filename).st_size/(MAX_SIZE)
    print filename, "will be split", split_times, "times"
    with h5py.File(filename, "r") as hf:
        data = hf["data"][:]
        odometry = hf["odometry"][:]
        odom_count = np.shape(odometry)[0]
        hsize = np.shape(data)[0]/odom_count

        max_new_odoms_count = odom_count / (split_times+1) + odom_count % (split_times+1)
        odoms_copied = 0
        for s in range(split_times+1):
            new_odoms_count = min(max_new_odoms_count, odom_count-odoms_copied)
            output_file = h5py.File(filename + ".%s"%(s), 'w')
            output_file.create_dataset('data', (new_odoms_count*hsize,) + np.shape(data)[1:], dtype='f4')
            output_file.create_dataset('odometry', (new_odoms_count,) + np.shape(odometry)[1:], dtype='f4')
            output_file['odometry'][...] = odometry[odoms_copied:odoms_copied+new_odoms_count]
            output_file['data'][...] = data[odoms_copied*hsize:(odoms_copied+new_odoms_count)*hsize]
            odoms_copied += new_odoms_count
