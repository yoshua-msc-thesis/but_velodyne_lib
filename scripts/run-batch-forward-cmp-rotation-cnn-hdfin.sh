#! /bin/bash

DEPLOY_DEF=${1:-/home/ivelas/workspace/ivelas-git/sw/cnn-generator/bash-definitions/odometry_l335_comp_rotation_hdfin_deploy.sh}
NET_LABEL=${2:-odometry_l335_rz_comp_hdfin_lr5e-4}
TRAIN_ITERATIONS=${3:-200000}
BATCH_SIZE=${4:-1}
SHIFTS=${5:-13}
MAX_ANGLE=${6:-1.2}
ANGLE_ID=${7:-2}
FEAT_CHANNELS=${8:-3}
INPUT_FILE_LISTS=${9:-/media/files/cnn_velodyne_data/hdf_data/rz-by-compare_filelists}

CNN_GENERATOR_DIR=/home/ivelas/workspace/ivelas-git/sw/cnn-generator
TRAINED_MODEL=$CNN_GENERATOR_DIR/NETS/$NET_LABEL/net_snapshot_iter_$TRAIN_ITERATIONS.caffemodel
OUTPUT_DIR_BASE=/media/files/cnn_velodyne_data/results/
OUTPUT_DIR=$OUTPUT_DIR_BASE/$(($(ls $OUTPUT_DIR_BASE | egrep '^[[:digit:]]+' -o | sort -g | tail -n1)+1))-$NET_LABEL-it$TRAIN_ITERATIONS-window
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
POSES_DIR=/media/files/cnn_velodyne_data/poses
CAFFE=~/lib/caffe/build/tools/caffe

mkdir -p $OUTPUT_DIR

cp $DEPLOY_DEF $TRAINED_MODEL $OUTPUT_DIR

for seq in 08 09 10 00 01 02 03 04 05 06 07
do
	DEPLOY_PROTOTXT=$CNN_GENERATOR_DIR/deploy-definitions/$(basename $DEPLOY_DEF).prototxt
	OUTPUT_FILE=$OUTPUT_DIR/$seq.txt
	bash $DEPLOY_DEF $OUTPUT_FILE $BATCH_SIZE $SHIFTS $MAX_ANGLE $ANGLE_ID $FEAT_CHANNELS | $CNN_GENERATOR_DIR/buildNetwork.sh > $DEPLOY_PROTOTXT
	expected_poses=$(wc -l < $POSES_DIR/$seq.txt)
	iterations=$((($expected_poses-2+$BATCH_SIZE)/$BATCH_SIZE))
	pushd $OUTPUT_DIR
		cp $INPUT_FILE_LISTS/$seq.files test.files
		$CAFFE test -model $DEPLOY_PROTOTXT -iterations $iterations -weights $TRAINED_MODEL -gpu 0
	popd
	head -n $expected_poses $OUTPUT_FILE > out_tmp; mv out_tmp $OUTPUT_FILE
done |& tee $OUTPUT_DIR/forward.log
