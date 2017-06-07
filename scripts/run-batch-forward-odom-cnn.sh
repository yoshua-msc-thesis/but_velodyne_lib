#! /bin/bash

DEPLOY_PROTOTXT=${1:-~/workspace/ivelas-git/sw/cnn-generator/deploy-definitions/odometry_l335_b1_shifts56_rcls_comp_deploy_fixshift.prototxt}
NET_LABEL=${2:-odometry_l335_b1_shifts56_rcls_comp_fixshift}
TRAINED_MODEL=/home/ivelas/workspace/ivelas-git/sw/cnn-generator/NETS/$NET_LABEL/net_snapshot_iter_200000.caffemodel
OUTPUT_DIR=/media/files/cnn_velodyne_data/results/13-$NET_LABEL-it200k-window

#DATA_DIR=/media/kitti/dataset_odometry_velodyne_ground_fake_ann/00-10/
DATA_DIR=/media/files/cnn_velodyne_data/2dMat_seq_3600deg
POSES_DIR=/media/files/cnn_velodyne_data/poses
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

mkdir -p $OUTPUT_DIR

for i in $(ls $DATA_DIR)
do
	echo $i
	pushd $DATA_DIR/$i/velodyne
		$SCRIPT_DIR/odometry_forward.py -p $DEPLOY_PROTOTXT -m $TRAINED_MODEL $(ls *.npy | sort) -i $POSES_DIR/$i.txt -g $OUTPUT_DIR/$i.graph | tee $OUTPUT_DIR/$i.txt
	popd
done

cp $DEPLOY_PROTOTXT $TRAINED_MODEL $OUTPUT_DIR
cp $SCRIPT_DIR/odometry_forward.py $OUTPUT_DIR/schema.py
