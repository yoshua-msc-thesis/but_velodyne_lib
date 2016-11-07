#! /bin/bash

for res_folder in $@
do
	pushd $res_folder

	cp -r /media/kitti/dataset_odometry_velodyne_odom_cnn_data/results/empty-evaluation evaluation
	for i in *.txt
	do 
		echo $i
		~/workspace/but_velodyne_lib/scripts/merge_pose_files.py /media/kitti/dataset_odometry_poses/poses/$i $i > evaluation/results/ivelas/data/$i
	done
	cd evaluation
	~/workspace/tmp-ivelas/sw/kitti-odometry-devkit/evaluate_odometry ivelas

	popd
done
