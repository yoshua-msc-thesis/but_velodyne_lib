#! /usr/bin/env python

import numpy as np
import sys
import math
import caffe
import cv_yaml
import argparse
import os

from odometry_cnn_data import horizontal_split
from odometry_cnn_data import schema_to_dic
from odometry_cnn_data import odom_deg_to_rad
from odometry_cnn_data import dof2matrix, matrix2dof
from eulerangles_lib import angle_axis_denorm2eulerXYZ
from kitti_poses_rot_quantization import classes_blob2angles 

BATCH_SCHEMA_DATA = [[3, 0],
                     [4, 1],
                     [5, 2],
                     [6, 3],
                     [7, 4],
                     [8, 5],
                     [9, 6],
                     [10, 7],
                     [11, 8],
                     [12, 9],
                     [13, 10],
                     [14, 11],
                     [15, 12],
                     [16, 13],
                     [17, 14],
                     [18, 15],

                     [3, 1],
                     [4, 2],
                     [5, 3],
                     [6, 4],
                     [7, 5],
                     [8, 6],
                     [9, 7],
                     [10, 8],
                     [11, 9],
                     [12, 10],
                     [13, 11],
                     [14, 12],
                     [15, 13],
                     [16, 14],
                     [17, 15],
                     [18, 16],

                     [3, 2],
                     [4, 3],
                     [5, 4],
                     [6, 5],
                     [7, 6],
                     [8, 7],
                     [9, 8],
                     [10, 9],
                     [11, 10],
                     [12, 11],
                     [13, 12],
                     [14, 13],
                     [15, 14],
                     [16, 15],
                     [17, 16],
                     [18, 17],

                     [2, 0],
                     [3, 1],
                     [4, 2],
                     [5, 3],
                     [6, 4],
                     [7, 5],
                     [8, 6],
                     [9, 7],
                     [10, 8],
                     [11, 9],
                     [12, 10],
                     [13, 11],
                     [14, 12],
                     [15, 13],
                     [16, 14],
                     [17, 15],

                     [2, 1],
                     [3, 2],
                     [4, 3],
                     [5, 4],
                     [6, 5],
                     [7, 6],
                     [8, 7],
                     [9, 8],
                     [10, 9],
                     [11, 10],
                     [12, 11],
                     [13, 12],
                     [14, 13],
                     [15, 14],
                     [16, 15],
                     [17, 16],

                     [1, 0],
                     [2, 1],
                     [3, 2],
                     [4, 3],
                     [5, 4],
                     [6, 5],
                     [7, 6],
                     [8, 7],
                     [9, 8],
                     [10, 9],
                     [11, 10],
                     [12, 11],
                     [13, 12],
                     [14, 13],
                     [15, 14],
                     [16, 15]]
BATCH_SCHEMA_ODOM = [[3],
                     [4],
                     [5],
                     [6],
                     [7],
                     [8],
                     [9],
                     [10],
                     [11],
                     [12],
                     [13],
                     [14],
                     [15],
                     [16],
                     [17],
                     [18]]

BATCH_SIZE = len(BATCH_SCHEMA_ODOM)
HISTORY_SIZE = len(BATCH_SCHEMA_DATA)/BATCH_SIZE
JOINED_FRAMES = len(BATCH_SCHEMA_DATA[0])
JOINED_ODOMETRIES = len(BATCH_SCHEMA_ODOM[0])
FEATURES = 3
max_in_data_schema = max(reduce(lambda x,y: x+y,BATCH_SCHEMA_DATA))
first_in_odom_schema = max(BATCH_SCHEMA_ODOM[0])

ZNORM_MEAN = [0]*6
ZNORM_STD_DEV = [1]*6
CUMMULATE_ODOMS = 1
ODOMS_UNITS = "deg"
ROT_TYPE = "euler"  # "euler" or "axis-angle", "classes" or "class_y"
DOF_WEIGHTS = [1.0] * 6

# in case of ROT_TYPE == "classes"
ROTATIONS_BINS = [20,100,20]
MAX_ROTATIONS = [1.5,6.0,1.5]  # deg
WINDOW_PROB_SIZE = 7

FRAME_WIDTH=360
FRAME_HEIGHT=64

HORIZONTAL_DIVISION = 1  # divide into the 4 cells
HORIZONTAL_DIVISION_OVERLAY = 0  # 19deg    =>    128deg per divided frame
CHANNELS = FEATURES * HORIZONTAL_DIVISION

OUTPUT_LAYER_NAME = "out_odometry"

DOF_REQUIRED = 6
DOF_PREDICTED = 3
DOF_PREDICTED_FIRST = 3

def create_blob(input_files, schema_dic):
    blob = np.empty([BATCH_SIZE*HISTORY_SIZE, JOINED_FRAMES*CHANNELS, FRAME_HEIGHT, FRAME_WIDTH / HORIZONTAL_DIVISION + HORIZONTAL_DIVISION_OVERLAY * 2])
    for file_i in range(len(input_files)):

        if input_files[file_i].endswith(".npy"):
            file_data = np.load(input_files[file_i])
        else:
            file_data = cv_yaml.load(input_files[file_i], 'range-y-intensity')
        file_data = np.moveaxis(file_data, [0, 1, 2], [1, 2, 0])

        file_data = horizontal_split(file_data, HORIZONTAL_DIVISION, HORIZONTAL_DIVISION_OVERLAY, FEATURES, FRAME_HEIGHT, FRAME_WIDTH)
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
        Rt = dof2matrix(dof)
        self.m = np.dot(self.m, Rt)

    def getDof(self):
        return matrix2dof(self.m)

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

class Edge3D:
    def __init__(self, src_id, trg_id, dof6):
        self.srcId = src_id
        self.trgId = trg_id
        self.dof6 = dof6

    def __gt__(self, other):
        if self.srcId > other.srcId:
            return True
        elif self.srcId < other.srcId:
            return False
        else:
            return self.trgId > other.trgId

    def __str__(self):
        output = "EDGE3 %s %s " % (self.srcId, self.trgId)
        for d in self.dof6:
            output += "%s " % d
        output += "99.1304 -0.869565 -0.869565 -1.73913 -1.73913 -1.73913 "
        output +=          "99.13040 -0.869565 -1.73913 -1.73913 -1.73913 "
        output +=                    "99.13050 -1.73913 -1.73913 -1.73913 "
        output +=                              "96.5217 -3.47826 -3.47826 "
        output +=                                       "96.5217 -3.47826 "
        output +=                                               "96.52170"
        return output

# returns 59 from [[[59]]] 
def extract_single_number(multi_array):
    shape = np.shape(multi_array)
    assert len(shape) == sum(shape)
    output = multi_array
    for i in range(len(shape)):
        output = output[0]
    return output

def axis_angle_to_euler_rad(dof):
    return dof[0:3] + angle_axis_denorm2eulerXYZ(dof[3:6])

def extract_prediction(data_blobs, blob_i, slot_i):
    if ROT_TYPE == "classes":
        return odom_deg_to_rad([0]*3 + classes_blob2angles(data_blobs, blob_i, [0, 1, 2], ['rot_class_x', 'rot_class_y', 'rot_class_z'], 
                                                           MAX_ROTATIONS, ROTATIONS_BINS, WINDOW_PROB_SIZE))
    if ROT_TYPE == "class_y":
        return odom_deg_to_rad([0]*3 + classes_blob2angles(data_blobs, blob_i, [1], ["rot_class_y"], 
                                                           MAX_ROTATIONS, ROTATIONS_BINS, WINDOW_PROB_SIZE))
    dof = [0]*DOF_REQUIRED
    for i in range(DOF_PREDICTED_FIRST, DOF_PREDICTED_FIRST+DOF_PREDICTED):
        dof[i] = extract_single_number(data_blobs[OUTPUT_LAYER_NAME][blob_i][slot_i*DOF_PREDICTED + i - DOF_PREDICTED_FIRST])
        dof[i] /= DOF_WEIGHTS[i]
    if ODOMS_UNITS == "deg":
        dof = odom_deg_to_rad(dof)
    if ROT_TYPE == "axis-angle":
        dof = axis_angle_to_euler_rad(dof)
    return [dof[i]*ZNORM_STD_DEV[i] + ZNORM_MEAN[i] for i in range(DOF_REQUIRED)]

parser = argparse.ArgumentParser(description="Forwarding CNN for odometry estimation")
parser.add_argument("-p", "--prototxt", dest="prototxt", type=str, required=True)
parser.add_argument("-m", "--caffemodel", dest="caffemodel", type=str, required=True)
parser.add_argument("-i", "--init-poses", dest="init_poses", type=str, required=True)
parser.add_argument("-g", "--out-graph", dest="out_graph", type=str, required=False)
parser.add_argument("feature_file", nargs='+', help='feature file', type=str)
args = parser.parse_args()

if len(args.feature_file) < max_in_data_schema+1:
    sys.stderr.write("ERROR: need at least %s feature files to feed CNN!\n", max_in_data_schema+1)
    sys.exit(1)

caffe.set_mode_gpu()
#caffe.set_mode_cpu()
net = caffe.Net(args.prototxt, args.caffemodel, caffe.TRAIN)

pose_graph = []

pose_counter = 0
for line in open(args.init_poses).readlines():
    pose = Pose(line.strip().split())
    print pose
    if pose_counter > 0:
        pose_graph.append(Edge3D(0, pose_counter, pose.getDof()))
    pose_counter += 1
    if pose_counter >= first_in_odom_schema:
        break

schema_dic = schema_to_dic(BATCH_SCHEMA_DATA)

firstFrameId = 0
while firstFrameId < len(args.feature_file):
    lastFrameId = min(firstFrameId + max_in_data_schema, len(args.feature_file)-1)
    blob = create_blob(args.feature_file[firstFrameId:lastFrameId+1], schema_dic)
    net.blobs['data'].data[...] = blob
    prediction = net.forward()
    effective_batch_size = compute_effective_batch_size(schema_dic, lastFrameId-firstFrameId+1)
    for b in range(effective_batch_size):
        dof = extract_prediction(prediction, b, 0)
        pose.move(dof)
        print pose
        for slot in range(len(BATCH_SCHEMA_ODOM[b])):
            trg_vertex = firstFrameId + BATCH_SCHEMA_ODOM[b][slot]
            src_vertex = trg_vertex - CUMMULATE_ODOMS
            dof = extract_prediction(prediction, b, slot)
            pose_graph.append(Edge3D(src_vertex, trg_vertex, dof))
    firstFrameId += BATCH_SIZE

if hasattr(args, "out_graph"):
    graph_file = open(args.out_graph, 'w')
    pose_graph.sort()
    for edge in pose_graph:
        graph_file.write("%s\n" % edge)
