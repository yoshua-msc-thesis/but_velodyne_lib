#! /bin/bash

DEPLOY_PROTOTXT=~/workspace/ivelas-git/sw/cnn-generator/deploy-definitions/odometry_l335_h6_b16_rot_classes_deploy.prototxt
TRAINED_MODEL=~/workspace/ivelas-git/sw/cnn-generator/NETS/odometry_l335_h6_b16_rot_classes_nostep/net_snapshot_iter_500000.caffemodel
OUTPUT_DIR=/media/files/cnn_velodyne_data/results/04-odometry_l335_h6_b16_rot_classes_nostep_it500k

#DATA_DIR=/media/kitti/dataset_odometry_velodyne_ground_fake_ann/00-10/
DATA_DIR=/media/files/cnn_velodyne_data/2dMat_seq
POSES_DIR=/media/files/cnn_velodyne_data/poses
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

mkdir -p $OUTPUT_DIR
#for i in $(ls $DATA_DIR)
for i in d3
do
	echo $i
	pushd $DATA_DIR/$i/velodyne
		$SCRIPT_DIR/odometry_forward.py -p $DEPLOY_PROTOTXT -m $TRAINED_MODEL $(ls *.yaml.gz | sort) -i $POSES_DIR/$i.txt -g $OUTPUT_DIR/$i.graph | tee $OUTPUT_DIR/$i.txt
	popd
done

cp $DEPLOY_PROTOTXT $TRAINED_MODEL $OUTPUT_DIR
cp $SCRIPT_DIR/odometry_forward.py $OUTPUT_DIR/schema.py
