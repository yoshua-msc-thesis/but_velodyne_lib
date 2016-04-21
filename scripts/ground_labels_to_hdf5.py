#! /usr/bin/env python

import sys
import h5py
import numpy
import cv

def load_from_yaml(yaml_filename, node_name):
    return numpy.asarray(cv.Load(yaml_filename, cv.CreateMemStorage(), node_name))

if len(sys.argv) < 3:
    sys.stderr.write("Invalid arguments. Usage: ground_labels_to_hdf5.py <input-YAML-file>* <output-HDF5-file>\n")
    sys.exit(1)

output_file = h5py.File(sys.argv[-1], 'w')
data_files = sys.argv[1:-1]
output_file.create_dataset('data', (len(data_files), 64, 360, 3), dtype='f8')
output_file.create_dataset('label', (len(data_files), 64, 360, 1), dtype='f4')

i = 0
for data_file in data_files:
    data = numpy.empty([64, 360, 3])
    labels = numpy.empty([64, 360, 1])
    
    data[..., 0] = load_from_yaml(data_file, 'range')
    data[..., 1] = load_from_yaml(data_file, 'y')
    data[..., 2] = load_from_yaml(data_file, 'intensity')
    labels[..., 0] = load_from_yaml(data_file, 'ground_labels')
    
    output_file['data'][i] = data
    output_file['label'][i] = labels
    i += 1

output_file.close()
