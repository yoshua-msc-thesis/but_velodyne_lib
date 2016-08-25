#! /usr/bin/env python

import numpy as np
import sys
import math
import caffe
import cv_yaml
import argparse
import eulerangles

HISTORY_SIZE = 5
BATCH_SIZE = 5
JOINED_FRAMES = 3
FEATURES = 3

def create_blob(input_files):
    blob = np.empty([25, 9, 64, 360])
    for file_i in range(len(input_files)):
        file_data = [cv_yaml.load(input_files[file_i], label) for label in ('range', 'y', 'intensity')]
        for feature_i in range(JOINED_FRAMES):
            history_i = file_i - feature_i
            slot_i = JOINED_FRAMES-1-feature_i
            if 0 <= history_i < HISTORY_SIZE and 0 <= slot_i < JOINED_FRAMES:
                blob_i = history_i*HISTORY_SIZE
                for ch_i in range(FEATURES):
                    blob[blob_i][slot_i*FEATURES + ch_i] = file_data[ch_i]
    return blob

class Pose:
    def __init__(self, kitti_pose = None):
        self.m = np.eye(4, 4)
        if kitti_pose is not None:
            assert len(kitti_pose) == 12
            kitti_pose = map(float, kitti_pose)
            for i in range(12):
                self.m[i/4, i%4] = kitti_pose[i]
    
    def move(self, dof):
        assert len(dof) == 6
        Rt = np.eye(4, 4)
        Rt[:3, :3] = eulerangles.euler2matXYZ(dof[3], dof[4], dof[5])
        for row in range(3):
            Rt[row, 3] = dof[row]
        self.m = np.dot(self.m, Rt)
        
    def __str__(self):
        output = ""
        for i in range(12):
            output += str(self.m[i/4, i%4]) + " "
        return output[:-1]

parser = argparse.ArgumentParser(description="Forwarding CNN for odometry estimation")
parser.add_argument("-p", "--prototxt", dest="prototxt", type=str, required=True)
parser.add_argument("-m", "--caffemodel", dest="caffemodel", type=str, required=True)
parser.add_argument("-i", "--init-poses", dest="init_poses", type=str)
parser.add_argument("feature_file", nargs='+', help='feature file', type=str)
args = parser.parse_args()

if len(args.feature_file) < HISTORY_SIZE+JOINED_FRAMES-1:
    sys.stderr.write("ERROR: need at least %s feature files to feed CNN!\n", HISTORY_SIZE+JOINED_FRAMES-1)
    sys.exit(1)

caffe.set_mode_gpu()
net = caffe.Net(args.prototxt, args.caffemodel, caffe.TRAIN)

if hasattr(args, "init_poses"):
    counter = 0
    for line in open(args.init_poses).readlines():
        pose = Pose(line.strip().split())
        print pose
        counter += 1
        if counter >= HISTORY_SIZE+JOINED_FRAMES-2:
            break
else:
    pose = Pose()

firstFrameId = 0
while firstFrameId + HISTORY_SIZE+JOINED_FRAMES-1 <= len(args.feature_file):
    blob = create_blob(args.feature_file[firstFrameId:firstFrameId+HISTORY_SIZE+JOINED_FRAMES-1])
    net.blobs['data'].data[...] = blob
    prediction = net.forward()
    dof = [0]*6
    for i in range(6):
        dof[i] = prediction["out_odometry"][0][i]
    pose.move(dof)
    print pose
    firstFrameId += 1
