#! /usr/bin/env python

import numpy as np
import sys
import math
import random
from numpy import dtype
import h5py
import cv
from __builtin__ import min
from eulerangles import mat2eulerZYX

def load_from_yaml(yaml_filename, node_name):
    return np.asarray(cv.Load(yaml_filename, cv.CreateMemStorage(), node_name))

class Odometry:
    def __init__(self, kitti_pose = [1, 0, 0, 0, 
                                     0, 1, 0, 0, 
                                     0, 0, 1, 0]):
        assert len(kitti_pose) == 12
        self.dof = [0]*6
        self.M = np.matrix([[0]*4, [0]*4, [0]*4, [0, 0, 0, 1]], dtype=np.float64)
        for i in range(12):
            self.M[i/4, i%4] = kitti_pose[i]
        self.setDofFromM()
    
    def setDofFromM(self):
        R = self.M[:3, :3]
        self.dof[0], self.dof[1], self.dof[2] = self.M[0, 3], self.M[1, 3], self.M[2, 3]
        self.dof[5], self.dof[4], self.dof[3] = mat2eulerZYX(R)
  
    def distanceTo(self, other):
        sq_dist = 0
        for i in range(3):
            sq_dist += (self.dof[i]-other.dof[i])**2
        return math.sqrt(sq_dist)

    def __mul__(self, other):
        out = Odometry()
        out.M = self.M * other.M
        out.setDofFromM()
        return out
    
    def __sub__(self, other):
        out = Odometry()
        out.M = np.linalg.inv(other.M) * self.M
        out.setDofFromM()
        return out

def gen_preserve_mask(poses, skip_prob):
    mask = [1]
    prev_pose = poses[0]
    current_pose = poses[1]
    for next_pose in poses[2:]:
        distance = next_pose.distanceTo(prev_pose)
        rndnum = random.random()
        if (distance < MAX_SPEED*0.1) and (rndnum < skip_prob):
            mask.append(0)
        else:
            mask.append(1)
            prev_pose = current_pose
        current_pose = next_pose
    mask.append(1)
    return mask

def mask_list(list, mask):
    if len(list) != len(mask):
        sys.stderr.write("Number of poses (%s) and velodyne frames (%s) differ!\n"%(len(mask), len(list)))
    output = []
    for i in range(min(len(mask), len(list))):
        if mask[i] != 0:
            output.append(list[i])
    return output

def get_delta_odometry(odometries, mask):
    if len(odometries) != len(mask):
        sys.stderr.write("Number of poses (%s) and velodyne frames (%s) differ!\n"%(len(mask), len(odometries)))
    output = [Odometry()]
    last_i = 0
    for i in range(1, min(len(mask), len(odometries))):
        if mask[i] != 0:
            output.append(odometries[i] - odometries[last_i])
            last_i = i
    return output
            
class OutputFiles:
    def __init__(self, batch_size, frames_to_join, output_prefix, max_seq_len):
        self.batchSize = batch_size
        self.framesToJoin = frames_to_join
        self.outputPrefix = output_prefix
        self.maxFramesPerFile = max_seq_len
        self.outFileSeqIndex = -1
    
    def newSequence(self, frames_count):
        self.framesCount = frames_count
        self.framesToWriteCount = (frames_count - self.batchSize + 1)
        self.outFileSeqIndex += 1
        self.out_files = []
        out_files_count = self.framesToWriteCount/self.maxFramesPerFile + 1 if self.framesToWriteCount%self.maxFramesPerFile > 0 else 0
        for split_index in range(out_files_count):
            if (split_index+1)*self.maxFramesPerFile <= self.framesToWriteCount:
                frames_in_file = self.maxFramesPerFile  
            else:
                frames_in_file = self.framesToWriteCount%self.maxFramesPerFile
            new_output_file = h5py.File(self.outputPrefix + "." + str(self.outFileSeqIndex) + "." + str(split_index) + ".hdf5", 'w')
            new_output_file.create_dataset('data', (frames_in_file*self.batchSize, 3*self.framesToJoin, 64, 360), dtype='f8')
            new_output_file.create_dataset('odometry', (frames_in_file, 6), dtype='f8')
            self.out_files.append(new_output_file)
    
    def putData(self, db_name, index, data):
        multiply = 1 if (db_name == 'odometry') else 5
        file_index = index/(self.maxFramesPerFile*multiply)
        self.out_files[file_index][db_name][index%(self.maxFramesPerFile*multiply)] = data

    def close(self):
        for f in self.out_files:
            f.close()

FRAMES_TO_JOIN=3
BATCH_SIZE=5
MIN_SKIP_PROB=0.0
MAX_SKIP_PROB=0.01
STEP_SKIP_PROB=0.9
MAX_SPEED=60/3.6
FILES_PER_HDF5=200

if len(sys.argv) < 2+FRAMES_TO_JOIN:
    sys.stderr.write("Expected arguments: <pose-file> <out-file-prefix> <frames.yaml>^{%s+}\n"%FRAMES_TO_JOIN)
    sys.exit(1)

poses_6dof = []
for line in open(sys.argv[1]).readlines():
    kitti_pose = map(float, line.strip().split())
    o = Odometry(kitti_pose)
    poses_6dof.append(o)

random.seed()
skip_prob = MIN_SKIP_PROB
out_files = OutputFiles(BATCH_SIZE, FRAMES_TO_JOIN, sys.argv[2], FILES_PER_HDF5)
while skip_prob < MAX_SKIP_PROB:
    mask = gen_preserve_mask(poses_6dof, skip_prob)
    frames = sum(mask)-FRAMES_TO_JOIN+1
    
    print "skip_prob:", skip_prob, "frames:", frames, "of:", len(mask)
    
    # TODO - maybe also duplication = no movement
    
    out_files.newSequence(frames)

    files_to_use = mask_list(sys.argv[3:], mask)
    odometry_to_use = get_delta_odometry(poses_6dof, mask)

    data_folded = [np.empty([9, 64, 360]) for i in range(FRAMES_TO_JOIN)]
    for i in range(len(files_to_use)):
        data_i = np.empty([3, 64, 360])
        odometry = np.asarray(odometry_to_use[i].dof)
        
        odom_out_index = i - FRAMES_TO_JOIN - BATCH_SIZE + 2
        if odom_out_index >= 0:
            out_files.putData('odometry', odom_out_index, odometry)
            
        data_i[0] = load_from_yaml(files_to_use[i], 'range')
        data_i[1] = load_from_yaml(files_to_use[i], 'y')
        data_i[2] = load_from_yaml(files_to_use[i], 'intensity')
        
        for bias in range(FRAMES_TO_JOIN):
            for slot in range(3):
                data_folded[bias][bias*3+slot] = data_i[slot]
        
        if i >= FRAMES_TO_JOIN-1:
            for h in range(BATCH_SIZE):
                out_index = ((i-FRAMES_TO_JOIN+1) - h)*BATCH_SIZE + h
                if 0 <= out_index < (frames-BATCH_SIZE+1)*BATCH_SIZE:
                    out_files.putData('data', out_index, data_folded[0])
        for j in range(FRAMES_TO_JOIN-1):
            data_folded[j] = np.copy(data_folded[j+1])

        #sys.stderr.write(".")
        if i%FILES_PER_HDF5 == 0 and i > 0:
            #sys.stderr.write("\n")
            print i, "/", len(files_to_use)
    
    skip_prob += STEP_SKIP_PROB
    out_files.close()
