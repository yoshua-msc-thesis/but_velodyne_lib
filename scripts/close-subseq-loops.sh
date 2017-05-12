#! /bin/bash

BUT_SCRIPTS=~/workspace/but_velodyne_lib/scripts
BUT_BINS=~/workspace/but_velodyne_lib/bin

$BUT_SCRIPTS/poses_to_graph.py < 03-poses-cls-m10.txt > 04-cls-subseq.unclosed.graph
cat loop.txt >>04-cls-subseq.unclosed.graph

# $BUT_BINS/cls-reg-subsequences --source_clouds_list forward.1.clouds --source_poses_file forward.1.poses --target_clouds_list backward.9.clouds --target_poses_file backward.9.poses --max_time_for_registration 1000 --max_iterations 1000 -g 2 -p 1 --matching_threshold NO_THRESHOLD | tee forward.1-to-backward.9.poses
$BUT_SCRIPTS/pose_to_edge.py --src_index_from 253 --src_index_to 495 --trg_index_from 1849 --trg_index_to 2120 -p 03-poses-cls-m10.txt -r forward.1-to-backward.9.poses -g 04-cls-subseq.unclosed.graph >>04-cls-subseq.unclosed.graph
$BUT_SCRIPTS/pose_to_edge.py --src_index_from 754 --src_index_to 887 --trg_index_from 1849 --trg_index_to 2120 -p 03-poses-cls-m10.txt -r forward.3-to-backward.9.poses -g 04-cls-subseq.unclosed.graph >>04-cls-subseq.unclosed.graph
$BUT_SCRIPTS/pose_to_edge.py --src_index_from 888 --src_index_to 1174 --trg_index_from 1741 --trg_index_to 1860 -p 03-poses-cls-m10.txt -r forward.4-to-backward.8.poses -g 04-cls-subseq.unclosed.graph >>04-cls-subseq.unclosed.graph
$BUT_SCRIPTS/pose_to_edge.py --src_index_from 0 --src_index_to 252 --trg_index_from 1849 --trg_index_to 2120 -p 03-poses-cls-m10.txt -r forward.0-to-backward.9.poses -g 04-cls-subseq.unclosed.graph >>04-cls-subseq.unclosed.graph

~/apps/SLAM_plus_plus_v2.10/bin/slam_plus_plus -i 04-cls-subseq.unclosed.graph --pose-only --no-detailed-timing
$BUT_BINS/slampp-solution-to-poses < solution.txt > 04-cls-subseq.closed.txt
$BUT_BINS/build-3d-model -p 04-cls-subseq.closed.txt $(ls fixed-by-03-poses-cls-m10/*.pcd | sort) -o 04-cls-subseq.closed.pcd
pcl_viewer 04-cls-subseq.closed.pcd.rgb.pcd
