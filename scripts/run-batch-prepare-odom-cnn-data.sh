#! /bin/bash

DATA_LABEL=joined2_hist3_batch4_skipped_and_succ_rad_cum3

OUT_DIR=/media/kitti/dataset_odometry_velodyne_odom_cnn_data/$DATA_LABEL
MATYLDA_OUT_DIR=/mnt/matylda1/ivelas/kitti/dataset_odometry_velodyne_odom_cnn_data/$DATA_LABEL

SCRIPTS_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
POSES=/media/kitti/dataset_odometry_poses/poses
VELODYNE_SEQ=/media/kitti/dataset_odometry_velodyne_ground_fake_ann/00-10

mkdir -p $OUT_DIR
rm -f $OUT_DIR/*
ssh merlin "mkdir -p $MATYLDA_OUT_DIR; rm -f $MATYLDA_OUT_DIR/*"

cp $SCRIPTS_DIR/odometry_data_to_hdf5_by_schema.py $OUT_DIR/schema.txt
scp $OUT_DIR/schema.txt merlin:$MATYLDA_OUT_DIR

for i in 00 01 02 03 04 05 06 07 08 09 10; do 
	echo $i
	$SCRIPTS_DIR/odometry_data_to_hdf5_by_schema.py $POSES/$i.txt $OUT_DIR/$i $(ls $VELODYNE_SEQ/$i/velodyne/*.gz | sort)
	scp $OUT_DIR/$i* merlin:$MATYLDA_OUT_DIR &
done

wait
