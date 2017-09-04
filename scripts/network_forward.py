#! /usr/bin/env python

import numpy as np
import sys
import math
import caffe
import argparse
import time

BATCH_SIZE = 4

def run(iterations):
    net = caffe.Net(args.prototxt, args.caffemodel, caffe.TRAIN)
    data_shape = net.blobs['data'].data.shape

    start = time.time()
    for i in range(iterations):
        net.blobs['data'].data[...] = np.random.rand(*data_shape)
        net.forward()

    return (time.time() - start) / iterations / BATCH_SIZE


parser = argparse.ArgumentParser(description="Forwarding CNN for odometry estimation")
parser.add_argument("-p", "--prototxt", dest="prototxt", type=str, required=True)
parser.add_argument("-m", "--caffemodel", dest="caffemodel", type=str, required=True)
args = parser.parse_args()

caffe.set_mode_gpu()
print "GPU", run(100)

caffe.set_mode_cpu()
print "CPU", run(10)
