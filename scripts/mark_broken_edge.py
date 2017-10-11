#! /usr/bin/env python

# ~/workspace/but_velodyne_lib/scripts/poses_to_graph.py < 03-poses-cls-m10.txt > 04-cls-subseq.unclosed.graph; ~/workspace/but_velodyne_lib/scripts/pose_to_edge.py --src_index_from 253 --src_index_to 495 --trg_index_from 1849 --trg_index_to 2120 -p 03-poses-cls-m10.txt -r forward.1-to-backward.9.poses -g 04-cls-subseq.unclosed.graph >>04-cls-subseq.unclosed.graph; cat loop.txt >>04-cls-subseq.unclosed.graph; ~/apps/SLAM_plus_plus_v2.10/bin/slam_plus_plus -i 04-cls-subseq.unclosed.graph --pose-only --no-detailed-timing; ~/workspace/but_velodyne_lib/bin/slampp-solution-to-poses < solution.txt > 04-cls-subseq.closed.txt; ~/workspace/but_velodyne_lib/bin/build-3d-model -p 04-cls-subseq.closed.txt $(ls fixed-by-03-poses-cls-m10/*.pcd | sort) -o 04-cls-subseq.closed.pcd; pcl_viewer 04-cls-subseq.closed.pcd.rgb.pcd 

import sys
import argparse

#PRECISION_MAT_COEFF = "8.3607 -1.6393 -1.6393 -1.6393 -1.6393 -1.6393\
# 8.3607 -1.6393 -1.6393 -1.6393 -1.6393\
# 8.3607 -1.6393 -1.6393 -1.6393\
# 8.3607 -1.6393 -1.6393\
# 8.3607 -1.6393\
# 8.3607"

#PRECISION_MAT_COEFF = "66.9970  -33.0030  -33.0030   -3.3003   -3.3003   -3.3003\
#66.9970  -33.0030   -3.3003   -3.3003   -3.3003\
#66.9970   -3.3003   -3.3003   -3.3003\
#99.6700   -0.3300   -0.3300\
#99.6700   -0.3300\
#99.6700"

PRECISION_MAT_COEFF = "\
0.0002 0 0 0 0 0 \
0.0002 0 0 0 0 \
0.0002 0 0 0 \
0.0005 0 0 \
0.0005 0 \
0.0005"

if len(sys.argv) < 3:
    sys.stderr.write("ERROR, expected arguments: <first-ixd> <last-idx>\n")
    sys.exit(1)

first_idx = int(sys.argv[1])
last_idx = int(sys.argv[2])

for line in sys.stdin.readlines():
    tokens = line.split()
    src_tokens = int(tokens[1])
    trg_tokens = int(tokens[2])
    if first_idx <= src_tokens == trg_tokens-1 < last_idx:
        prefix = " ".join(tokens[0:9])
        print prefix + " " + PRECISION_MAT_COEFF
    else:
        print line.strip()
