#! /bin/bash

DATA_LABEL=j2_h1_b3_deg_rcls96

OUT_DIR=/media/files/cnn_velodyne_data/hdf_data/$DATA_LABEL
MATYLDA_OUT_DIR=/mnt/matylda1/ivelas/cnn_velodyne_data/hdf_data/$DATA_LABEL
POSES=/media/files/cnn_velodyne_data/poses
VELODYNE_SEQ=/media/files/cnn_velodyne_data/2dMat_seq_3600deg

SCRIPTS_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

mkdir -p $OUT_DIR
rm -f $OUT_DIR/*
ssh merlin "mkdir -p $MATYLDA_OUT_DIR; rm -f $MATYLDA_OUT_DIR/*"

cp $SCRIPTS_DIR/odometry_data_to_hdf5_by_schema.py $OUT_DIR/schema.txt
scp $OUT_DIR/schema.txt merlin:$MATYLDA_OUT_DIR

for i in $(ls $VELODYNE_SEQ); do
	echo "Processing sequence: $i"
	pushd $VELODYNE_SEQ/$i/velodyne
		$SCRIPTS_DIR/odometry_data_to_hdf5_by_schema.py $POSES/$i.txt $OUT_DIR/$i $(ls *.gz | sort)
	popd
	for hdf_file in $OUT_DIR/$i*.hdf5; do
		echo "Adding rotation classes to $hdf_file"
		$SCRIPTS_DIR/kitti_poses_rot_quantization_yawcomp.py $hdf_file
		mv $hdf_file.rotclasses $hdf_file
		scp $hdf_file merlin:$MATYLDA_OUT_DIR &
	done
done |& tee $OUT_DIR/output.log

wait
