#! /bin/bash

INPUT_DATA_DIR=/media/kitti/dataset_odometry_velodyne_odom_cnn_data/joined2_hist3_batch4_skipped_and_succ_deg_aligned
SUFFIX=shuffled_balanced
OUTPUT_DATA_DIR=${INPUT_DATA_DIR}_${SUFFIX}
MATYLDA_OUT_DIR=/mnt/matylda1/ivelas/kitti/dataset_odometry_velodyne_odom_cnn_data/$(basename $INPUT_DATA_DIR)_${SUFFIX}

SCRIPTS_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
POSES=/media/kitti/dataset_odometry_poses/poses

mkdir -p $OUTPUT_DATA_DIR
rm -f $INPUT_DATA_DIR/*.shuffled $OUTPUT_DATA_DIR/*
ssh merlin "mkdir -p $MATYLDA_OUT_DIR; rm -f $MATYLDA_OUT_DIR/*"

for i in $INPUT_DATA_DIR/*.hdf5; do 
	echo $i
	cat $POSES/*.txt | $SCRIPTS_DIR/kitti_poses_rot_histogram.py $i
	target=$OUTPUT_DATA_DIR/$(basename $i)
	mv $i.shuffled $target
	scp $target merlin:$MATYLDA_OUT_DIR &
done

wait
