#! /usr/bin/env python

import sys
import math
import numpy as np

from odometry_cnn_data import load_kitti_poses
from odometry_cnn_data import get_delta_odometry
from odometry_cnn_data import Odometry
from numpy import inf

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

odoms = get_delta_odometry(load_kitti_poses(sys.stdin))

for i in range(1, len(odoms)):
    print Edge3D(i-1, i, odoms[i].dof)
