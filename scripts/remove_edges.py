#! /usr/bin/env python

# ~/workspace/but_velodyne_lib/scripts/poses_to_graph.py < 03-poses-cls-m10.txt > 04-cls-subseq.unclosed.graph; ~/workspace/but_velodyne_lib/scripts/pose_to_edge.py --src_index_from 253 --src_index_to 495 --trg_index_from 1849 --trg_index_to 2120 -p 03-poses-cls-m10.txt -r forward.1-to-backward.9.poses -g 04-cls-subseq.unclosed.graph >>04-cls-subseq.unclosed.graph; cat loop.txt >>04-cls-subseq.unclosed.graph; ~/apps/SLAM_plus_plus_v2.10/bin/slam_plus_plus -i 04-cls-subseq.unclosed.graph --pose-only --no-detailed-timing; ~/workspace/but_velodyne_lib/bin/slampp-solution-to-poses < solution.txt > 04-cls-subseq.closed.txt; ~/workspace/but_velodyne_lib/bin/build-3d-model -p 04-cls-subseq.closed.txt $(ls fixed-by-03-poses-cls-m10/*.pcd | sort) -o 04-cls-subseq.closed.pcd; pcl_viewer 04-cls-subseq.closed.pcd.rgb.pcd 

import sys

if len(sys.argv) < 3:
    sys.stderr.write("ERROR, expected arguments: <first-ixd> <last-idx>\n")
    sys.exit(1)

first_idx = int(sys.argv[1])
last_idx = int(sys.argv[2])

for line in sys.stdin.readlines():
    tokens = line.split()
    src_vertex = int(tokens[1])
    trg_vertex = int(tokens[2])
    if (src_vertex < first_idx or src_vertex > last_idx) and (trg_vertex < first_idx or trg_vertex > last_idx):
        sufix = " ".join(tokens[3:])
        if trg_vertex > last_idx:
            trg_vertex -= last_idx-first_idx+1
        if src_vertex > last_idx:
            src_vertex -= last_idx-first_idx+1
        print tokens[0], src_vertex, trg_vertex, sufix
