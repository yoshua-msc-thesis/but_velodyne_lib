#! /bin/bash

INPUT_DATA_DIR=/media/kitti/dataset_odometry_velodyne_odom_cnn_data/joined2_hist3_batch4_skipped_and_succ_deg_aligned_prob8e-1_shuffled
SUFFIX=rotclasses
OUTPUT_DATA_DIR=${INPUT_DATA_DIR}_${SUFFIX}
MATYLDA_OUT_DIR=/mnt/matylda1/ivelas/kitti/dataset_odometry_velodyne_odom_cnn_data/$(basename $INPUT_DATA_DIR)_${SUFFIX}

SCRIPTS_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

mkdir -p $OUTPUT_DATA_DIR
rm -f $INPUT_DATA_DIR/*.rotclasses $OUTPUT_DATA_DIR/*
ssh merlin "mkdir -p $MATYLDA_OUT_DIR; rm -f $MATYLDA_OUT_DIR/*"

for i in $INPUT_DATA_DIR/*.hdf5*; do 
	echo $i
	$SCRIPTS_DIR/kitti_poses_rot_quantization.py $i
	target=$OUTPUT_DATA_DIR/$(basename $i)
	(scp $i.rotclasses merlin:$MATYLDA_OUT_DIR; rm $i.rotclasses) &
	#mv $i.rotclasses $target
	#scp $target merlin:$MATYLDA_OUT_DIR &
done

wait
