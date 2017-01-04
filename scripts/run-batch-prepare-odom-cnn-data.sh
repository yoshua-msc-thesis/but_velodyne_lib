#! /bin/bash

AXIS=x
DATA_LABEL=j2_h1_b3_deg_r${AXIS}_cls12

OUT_DIR=/media/files/cnn_velodyne_data/hdf_data/$DATA_LABEL
MATYLDA_OUT_DIR=/mnt/matylda1/ivelas/cnn_velodyne_data/hdf_data/$DATA_LABEL
POSES=/media/files/cnn_velodyne_data/poses
VELODYNE_SEQ=/media/files/cnn_velodyne_data/2dMat_seq_1800d_anglecomp

SCRIPTS_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

mkdir -p $OUT_DIR
rm -f $OUT_DIR/*
ssh merlin "mkdir -p $MATYLDA_OUT_DIR; rm -f $MATYLDA_OUT_DIR/*"

cp $SCRIPTS_DIR/odometry_data_to_hdf5_by_schema.py $OUT_DIR/schema.txt
scp $OUT_DIR/schema.txt merlin:$MATYLDA_OUT_DIR

for seq in 00 01 02 03 04 05 06 07 08 09 10; do
	echo "Processing sequence: $seq"
	pushd $VELODYNE_SEQ
		$SCRIPTS_DIR/odometry_data_to_hdf5_by_schema.py $POSES/$seq.txt $OUT_DIR/$seq $(find $(find . -name velodyne | grep "r${axis}_.*/$seq/velodyne") -name '*.gz' |
			sort --version-sort -k2 -t"/" | sort --version-sort --stable -k6 -t"/")
	popd
	for hdf_file in $OUT_DIR/$seq*.hdf5; do
		echo "Adding rotation classes to $hdf_file"
		$SCRIPTS_DIR/kitti_poses_rot_quantization.py $hdf_file
		mv $hdf_file.rotclasses $hdf_file
		scp $hdf_file merlin:$MATYLDA_OUT_DIR &
	done
done |& tee $OUT_DIR/output.log

wait
