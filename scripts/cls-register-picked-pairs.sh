#! /bin/bash

function get_batch {
	local i=$1
	local cnt=$2
	head -n $(($i+$cnt)) | tail -n $cnt	
}

function register_overlaping_frames {
	local proc_id=$1
	local proc_number=$2
	local frames=$3

	local src_clouds=$(mktemp)
	local trg_clouds=$(mktemp)
	local src_pose_file=$(mktemp)
	local trg_pose_file=$(mktemp)
	awk "(NR%$proc_number) == $(($proc_id-1))" $frames | while read pair
	do
		local src_i=$(awk '{print $1}'<<<$pair)
		local trg_i=$(awk '{print $2}'<<<$pair)
		if [ $src_i -gt $trg_i ]; then
			tmp=$src_i; src_i=$trg_i; trg_i=$tmp
		fi
		get_batch $src_i $CUMULATED_FRAMES < $INIT_POSES > $src_pose_file
		get_batch $trg_i $CUMULATED_FRAMES < $INIT_POSES > $trg_pose_file
		ls $CLOUDS_DIR/*.pcd | sort | get_batch $(($src_i*$sensors_cnt)) $(($CUMULATED_FRAMES*$sensors_cnt)) > $src_clouds
		ls $CLOUDS_DIR/*.pcd | sort | get_batch $(($trg_i*$sensors_cnt)) $(($CUMULATED_FRAMES*$sensors_cnt)) > $trg_clouds

		echo -n "$src_i $trg_i "
		$BUT_VELODYNE_LIB/bin/cls-reg-subsequences -c $CALIBRATION -g 10 -p 5 --max_time_for_registration 20 --max_iterations 5000 --target_error 0.001 \
			--matching_threshold VALUE_THRESHOLD --matching_threshold_value $threshold \
			--source_clouds_list $src_clouds --source_poses_file $src_pose_file --target_clouds_list $trg_clouds --target_poses_file $trg_pose_file
	done

	rm $src_clouds $trg_clouds $src_pose_file $trg_pose_file
}

if [ $# -ne 5 ]
then
	echo "ERROR, expected arguments: $0 <clouds-dir> <init-poses> <original-pose-graph> <calibration> <out-dir>" >&2
	exit 1
fi

CLOUDS_DIR=$1
INIT_POSES=$2
INIT_POSE_GRAPH=$3
CALIBRATION=$4
OUT_DIR=$5

CUMULATED_FRAMES=1
sensors_cnt=$(wc -l < $CALIBRATION)

pose_graph=$INIT_POSE_GRAPH
pose_file=$INIT_POSES
for i in $(seq 100)
do
	reg_pair=$OUT_DIR/pair-registered.$i
	$BUT_VELODYNE_LIB/bin/cls-registration-of-picked-frames --pose_file $pose_file -c $CALIBRATION $(ls $CLOUDS_DIR/*.pcd | sort) > $reg_pair
	new_pose_graph=$OUT_DIR/pair-registered.$i.graph
	$BUT_VELODYNE_LIB/scripts/poses_pairs_to_graph.py --poses $INIT_POSES --corrections $reg_pair --cumulated_frames $CUMULATED_FRAMES | cat $pose_graph - > $new_pose_graph
	pushd $OUT_DIR
		~/apps/SLAM_plus_plus_v2.10/bin/slam_plus_plus -i $new_pose_graph
	popd
	new_pose_file=$OUT_DIR/pair-registered.$i.poses
	$BUT_VELODYNE_LIB/bin/slampp-solution-to-poses < $OUT_DIR/solution.txt > $new_pose_file
	pose_file=$new_pose_file
	pose_graph=$new_pose_graph
done

$BUT_VELODYNE_LIB/bin/build-3d-model -p $pose_file -s1.0 -o $OUT_DIR/pairs-registered.pcd --sensor_poses $CALIBRATION $(ls $CLOUDS_DIR/*.pcd | sort)
pdal translate $OUT_DIR/pairs-registered.pcd $OUT_DIR/pairs-registered.laz
