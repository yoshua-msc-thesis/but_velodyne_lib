#! /usr/bin/env python

import numpy as np
import sys
import math
import random
import h5py
import cv_yaml

if len(sys.argv) != 1:
    sys.stderr.write("ERROR, arguments: [output-file.hdf5]\n")
    sys.exit(1)

lines = sys.stdin.readlines()
data = []
for l in lines:
    tokens = l.strip().split()
    odometry = map(float, tokens[0:6])
    angle_classes = map(int, tokens[6:9])
    files = tokens[9:]
    data.append({"odometry":odometry, "rot_class_x":angle_classes[0], 
                 "rot_class_y":angle_classes[1], "rot_class_z":angle_classes[1],
                 "data":files})

dato = np.load(data[0]["data"][0])
rings, width, channels = dato.shape
frames_per_odometry = len(data[0]["data"])

with h5py.File(sys.argv[1], 'w') as output_file:
    output_file.create_dataset('data', (len(data), channels*frames_per_odometry, rings, width), dtype='f4')
    output_file.create_dataset('odometry',    (len(data), 6), dtype='f4')
    output_file.create_dataset('rot_class_x', (len(data),), dtype='f4')
    output_file.create_dataset('rot_class_y', (len(data),), dtype='f4')
    output_file.create_dataset('rot_class_z', (len(data),), dtype='f4')

    for di, dato in enumerate(data):
        for fi, file in dato["data"]:
            frame = np.load(file)
            frame = np.moveaxis(frame, [0, 1, 2], [1, 2, 0])
            output_file["data"][di, fi*channels, ...] = frame
        output_file["odometry"][di, :] = np.array(dato["odometry"])
        output_file["rot_class_x"][di] = dato["rot_class_x"]
        output_file["rot_class_y"][di] = dato["rot_class_y"]
        output_file["rot_class_z"][di] = dato["rot_class_z"]
