#! /usr/bin/env python

import numpy as np
import sys
import math
import caffe
import cv_yaml
import argparse
import eulerangles

from odometry_cnn_data import horizontal_split

BATCH_SCHEMA_DATA = [[5, 0],
                     [6, 1],
                     [7, 2],
                     [8, 3],

                     [5, 1],
                     [6, 2],
                     [7, 3],
                     [8, 4],
                     
                     [5, 2],
                     [6, 3],
                     [7, 4],
                     [8, 5],

                     [5, 3],
                     [6, 4],
                     [7, 5],
                     [8, 6],

                     [5, 4],
                     [6, 5],
                     [7, 6],
                     [8, 7]]
BATCH_SCHEMA_ODOM = [5, 6, 7, 8]
BATCH_SIZE = len(BATCH_SCHEMA_ODOM)
HISTORY_SIZE = len(BATCH_SCHEMA_DATA)/BATCH_SIZE
JOINED_FRAMES = len(BATCH_SCHEMA_DATA[0])
FEATURES = 3
max_in_data_schema = max(reduce(lambda x,y: x+y,BATCH_SCHEMA_DATA))
min_in_odom_schema = min(BATCH_SCHEMA_ODOM)

ZNORM_MEAN = [0]*6
ZNORM_STD_DEV = [1]*6

HORIZONTAL_DIVISION = 1  # divide into the 4 cells
HORIZONTAL_DIVISION_OVERLAY = 0  # 19deg    =>    128deg per divided frame
CHANNELS = FEATURES * HORIZONTAL_DIVISION

def schema_to_dic(data_schema):
    data_dic = {i:[] for i in set(reduce(lambda x,y: x+y,data_schema))}
    for frame_i in range(len(data_schema)):
        for slot_i in range(len(data_schema[frame_i])):
            data_dic[data_schema[frame_i][slot_i]].append({"slot":slot_i, "frame":frame_i})

    return data_dic

def create_blob(input_files, schema_dic):
    blob = np.empty([BATCH_SIZE*HISTORY_SIZE, JOINED_FRAMES*CHANNELS, 64, 360 / HORIZONTAL_DIVISION + HORIZONTAL_DIVISION_OVERLAY * 2])
    for file_i in range(len(input_files)):
        file_data = np.empty([FEATURES, 64, 360])
        file_data[0] = cv_yaml.load(input_files[file_i], 'range')
        file_data[1] = cv_yaml.load(input_files[file_i], 'y')
        file_data[2] = cv_yaml.load(input_files[file_i], 'intensity')
        file_data = horizontal_split(file_data, HORIZONTAL_DIVISION, HORIZONTAL_DIVISION_OVERLAY, FEATURES)
        for indices in schema_dic[file_i]:
            for f in range(CHANNELS):
                blob[indices["frame"]][indices["slot"]*CHANNELS + f] = file_data[f]
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

def compute_effective_batch_size(schema_dic, frames):
    max_in_frames = map(max, BATCH_SCHEMA_DATA)
    require_for_batch_dato = [-1]*BATCH_SIZE
    for i in range(len(max_in_frames)):
        require_for_batch_dato[i%BATCH_SIZE] = max(require_for_batch_dato[i%BATCH_SIZE], max_in_frames[i])
    effect_bs = 0
    while effect_bs < BATCH_SIZE:
        if require_for_batch_dato[effect_bs] >= frames:
            break
        effect_bs += 1
    return effect_bs

parser = argparse.ArgumentParser(description="Forwarding CNN for odometry estimation")
parser.add_argument("-p", "--prototxt", dest="prototxt", type=str, required=True)
parser.add_argument("-m", "--caffemodel", dest="caffemodel", type=str, required=True)
parser.add_argument("-i", "--init-poses", dest="init_poses", type=str, required=True)
parser.add_argument("feature_file", nargs='+', help='feature file', type=str)
args = parser.parse_args()

if len(args.feature_file) < max_in_data_schema+1:
    sys.stderr.write("ERROR: need at least %s feature files to feed CNN!\n", max_in_data_schema+1)
    sys.exit(1)

caffe.set_mode_gpu()
net = caffe.Net(args.prototxt, args.caffemodel, caffe.TRAIN)

pose_counter = 0
for line in open(args.init_poses).readlines():
    pose = Pose(line.strip().split())
    print pose
    pose_counter += 1
    if pose_counter >= min_in_odom_schema:
        break

schema_dic = schema_to_dic(BATCH_SCHEMA_DATA)

firstFrameId = 0
while firstFrameId < len(args.feature_file):
    lastFrameId = min(firstFrameId + max_in_data_schema, len(args.feature_file)-1)
    blob = create_blob(args.feature_file[firstFrameId:lastFrameId+1], schema_dic)
    net.blobs['data'].data[...] = blob
    prediction = net.forward()
    dof = [0]*6
    effective_batch_size = compute_effective_batch_size(schema_dic, lastFrameId-firstFrameId+1)
    for b in range(effective_batch_size):
        for i in range(6):
            dof[i] = prediction["out_odometry"][b][i]*ZNORM_STD_DEV[i] + ZNORM_MEAN[i]
        pose.move(dof)
        print pose
    firstFrameId += BATCH_SIZE
