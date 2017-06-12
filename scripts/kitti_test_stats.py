#! /usr/bin/env python

import sys

KITTI_SEQ_LENGTHS = [
    4540,
    1100,
    4660,
     800,
     270,
    2760,
    1100,
    1100,
    4070,
    1590,
    1200
]

KITTI_TEST_SEQ = [8, 9, 10]

errors = {}
for line in sys.stdin.readlines():
    tokens = line.split()
    errors[tokens[0]] = tokens[1]

total_err = 0
total_len = 0
for i in KITTI_TEST_SEQ:
    seq_id = "%02d" % (i)
    total_err += float(errors[seq_id])*KITTI_SEQ_LENGTHS[i]
    total_len += KITTI_SEQ_LENGTHS[i]

print total_err / total_len
