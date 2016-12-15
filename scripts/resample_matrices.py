#! /usr/bin/env python

import sys
import math
import cv2
import cv_yaml
import numpy as np

DATA_LABELS = ["range", "y", "intensity"]

if len(sys.argv) != 3:
    sys.strderr.write("Invalid arguments, usage: %s <input.yaml> <output.yaml>\n", sys.argv[0])
    sys.exit(1)

for label in DATA_LABELS:
    src_mat = cv_yaml.load(sys.argv[1], label)
    src_shape = np.shape(src_mat)
    dst_shape = (src_shape[0], src_shape[1]/2.0)
    dst_mat = np.zeros(shape=dst_shape, dtype=np.float32)
    print np.shape(dst_mat[:,0])
    for col in range(int(dst_shape[1])):
        dst_mat[:,col] = (src_mat[:,2*col] + src_mat[:,2*col+1]) / 2.0    
    cv2.imshow("src", src_mat)
    cv2.imshow("dst", dst_mat)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
