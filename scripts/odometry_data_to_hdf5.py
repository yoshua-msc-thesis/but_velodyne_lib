#! /usr/bin/env python

import numpy as np
import sys
import math
import random
from numpy import dtype
import h5py
import cv
from __builtin__ import min


def mat2eulerZYX(M, cy_thresh=None):
    ''' Discover Euler angle vector from 3x3 matrix

    Uses the conventions above.

    Parameters
    ----------
    M : array-like, shape (3,3)
    cy_thresh : None or scalar, optional
       threshold below which to give up on straightforward arctan for
       estimating x rotation.  If None (default), estimate from
       precision of input.

    Returns
    -------
    z : scalar
    y : scalar
    x : scalar
       Rotations in radians around z, y, x axes, respectively

    Notes
    -----
    If there was no numerical error, the routine could be derived using
    Sympy expression for z then y then x rotation matrix, which is::

      [                       cos(y)*cos(z),                       -cos(y)*sin(z),         sin(y)],
      [cos(x)*sin(z) + cos(z)*sin(x)*sin(y), cos(x)*cos(z) - sin(x)*sin(y)*sin(z), -cos(y)*sin(x)],
      [sin(x)*sin(z) - cos(x)*cos(z)*sin(y), cos(z)*sin(x) + cos(x)*sin(y)*sin(z),  cos(x)*cos(y)]

    with the obvious derivations for z, y, and x

       z = atan2(-r12, r11)
       y = asin(r13)
       x = atan2(-r23, r33)

    Problems arise when cos(y) is close to zero, because both of::

       z = atan2(cos(y)*sin(z), cos(y)*cos(z))
       x = atan2(cos(y)*sin(x), cos(x)*cos(y))

    will be close to atan2(0, 0), and highly unstable.

    The ``cy`` fix for numerical instability below is from: *Graphics
    Gems IV*, Paul Heckbert (editor), Academic Press, 1994, ISBN:
    0123361559.  Specifically it comes from EulerAngles.c by Ken
    Shoemake, and deals with the case where cos(y) is close to zero:

    See: http://www.graphicsgems.org/

    The code appears to be licensed (from the website) as "can be used
    without restrictions".
    '''
    M = np.asarray(M)
    if cy_thresh is None:
        try:
            cy_thresh = np.finfo(M.dtype).eps * 4
        except ValueError:
            cy_thresh = _FLOAT_EPS_4
    r11, r12, r13, r21, r22, r23, r31, r32, r33 = M.flat
    # cy: sqrt((cos(y)*cos(z))**2 + (cos(x)*cos(y))**2)
    cy = math.sqrt(r33*r33 + r23*r23)
    if cy > cy_thresh: # cos(y) not close to zero, standard form
        z = math.atan2(-r12,  r11) # atan2(cos(y)*sin(z), cos(y)*cos(z))
        y = math.atan2(r13,  cy) # atan2(sin(y), cy)
        x = math.atan2(-r23, r33) # atan2(cos(y)*sin(x), cos(x)*cos(y))
    else: # cos(y) (close to) zero, so x -> 0.0 (see above)
        # so r21 -> sin(z), r22 -> cos(z) and
        z = math.atan2(r21,  r22)
        y = math.atan2(r13,  cy) # atan2(sin(y), cy)
        x = 0.0
    return z, y, x

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
            

FRAMES_TO_JOIN=3
HISTORY_SIZE=5
MIN_SKIP_PROB=0.0
MAX_SKIP_PROB=0.01
STEP_SKIP_PROB=0.1
MAX_SPEED=60/3.6

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
out_file_index = 0
while skip_prob < MAX_SKIP_PROB:
    mask = gen_preserve_mask(poses_6dof, skip_prob)
    frames = sum(mask)-FRAMES_TO_JOIN+1
    
    print "skip_prob:", skip_prob, "frames:", frames, "of:", len(mask)
    
    # TODO - maybe also duplication = no movement
    
    output_file = h5py.File(sys.argv[2] + "." + str(out_file_index) + ".hdf5", 'w')
    output_file_size = (frames-HISTORY_SIZE+1)*HISTORY_SIZE
    output_file.create_dataset('data', (output_file_size, 3*FRAMES_TO_JOIN, 64, 360), dtype='f8')
    output_file.create_dataset('odometry', (frames-HISTORY_SIZE+1, 6), dtype='f8')

    files_to_use = mask_list(sys.argv[3:], mask)
    odometry_to_use = get_delta_odometry(poses_6dof, mask)

    data_folded = [np.empty([9, 64, 360]) for i in range(FRAMES_TO_JOIN)]
    for i in range(len(files_to_use)):
        data_i = np.empty([3, 64, 360])
        odometry = np.asarray(odometry_to_use[i].dof)
        
        odom_out_index = i - FRAMES_TO_JOIN - HISTORY_SIZE + 2
        if odom_out_index >= 0:
            output_file['odometry'][odom_out_index] = odometry
            
        data_i[0] = load_from_yaml(files_to_use[i], 'range')
        data_i[1] = load_from_yaml(files_to_use[i], 'y')
        data_i[2] = load_from_yaml(files_to_use[i], 'intensity')
        
        for bias in range(FRAMES_TO_JOIN):
            for slot in range(3):
                data_folded[bias][bias*3+slot] = data_i[slot]
        
        if i >= FRAMES_TO_JOIN-1:
            for h in range(HISTORY_SIZE):
                out_index = ((i-FRAMES_TO_JOIN+1) - h)*HISTORY_SIZE + h
                if 0 <= out_index < output_file_size:
                    output_file['data'][out_index] = data_folded[0]
        for j in range(FRAMES_TO_JOIN-1):
            data_folded[j] = np.copy(data_folded[j+1])

        if i%100 == 0:
            print i, "/", len(files_to_use)

    output_file.close()
    
    skip_prob += STEP_SKIP_PROB
    out_file_index += 1
