#! /bin/bash

POSES_DIR=/media/files/cnn_velodyne_data/poses/
EMPTY_EVAL=/media/files/cnn_velodyne_data/results/empty-evaluation

for res_folder in $@
do
	pushd $res_folder

	rm -rf evaluation
	cp -r $EMPTY_EVAL evaluation
	for i in *.txt
	do 
		echo "Merging $i"
		~/workspace/but_velodyne_lib/scripts/merge_pose_files.py $POSES_DIR/$i $i > evaluation/results/ivelas/data/$i
	done
	cd evaluation
	~/workspace/tmp-ivelas/sw/kitti-odometry-devkit/evaluate_odometry ivelas

	popd
done
