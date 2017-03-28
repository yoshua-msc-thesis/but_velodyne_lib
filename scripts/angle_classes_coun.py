#! /usr/bin/env python

import numpy as np
import sys
import math
import random
from string import strip
import json

from odometry_cnn_data import Odometry
from odometry_cnn_data import load_kitti_poses
from odometry_cnn_data import get_delta_odometry
from kitti_poses_rot_quantization import angles2classes
from numpy_point_cloud import DEG_TO_RAD

CLASSES_COUNTS=[13, 56, 13]
MAX_ANGLES=[1.3*DEG_TO_RAD, 5.6*DEG_TO_RAD, 1.3*DEG_TO_RAD]

odometries = []
for file in sys.argv[1:]:
    odometries += get_delta_odometry(load_kitti_poses(file))

for odom in odometries:
    classes = angles2classes(odom.dof[3:], MAX_ANGLES, CLASSES_COUNTS)
    print classes[0], classes[1], classes[2]
