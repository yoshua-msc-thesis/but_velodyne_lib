#! /bin/bash

function get_batch {
	i=$1
	cnt=$2
	head -n $(($i+$cnt)) | tail -n $cnt	
}

if [ $# -ne 8 ]
then
	echo "ERROR, expected arguments: $0 <clouds-dir> <init-poses> <original-pose-graph> <pairs> <cumulated-frames> <threshold-pts-dist> <min-frame-idx-diff> <calibration>" >&2
	exit 1
fi

CLOUDS_DIR=$1
INIT_POSES=$2
POSE_GRAPH=$3
PAIRS=$4
CUMULATED_FRAMES=$5
THRESH=$6
MIN_IDX_DIFF=$7
CALIBRATION=$8
#CALIBRATION=/media/files/4recon/velodyne_data/2Velodynes-upgm-kancl/calibration/calibration.txt
#CALIBRATION=/media/files/4recon/identitiy.txt

sensors_cnt=$(wc -l < $CALIBRATION)

src_clouds=$(mktemp)
trg_clouds=$(mktemp)
src_pose_file=$(mktemp)
trg_pose_file=$(mktemp)
awk "\$3 < $THRESH" $PAIRS | while read pair
do
	src_i=$(awk '{print $1}'<<<$pair)
	trg_i=$(awk '{print $2}'<<<$pair)
	if [ $src_i -gt $trg_i ]
	then
		tmp=$src_i
		src_i=$trg_i
		trg_i=$tmp
	fi
	if [ $(python <<<"print abs($src_i-$trg_i)") -gt $MIN_IDX_DIFF ]
	then
		get_batch $src_i $CUMULATED_FRAMES < $INIT_POSES > $src_pose_file
		get_batch $trg_i $CUMULATED_FRAMES < $INIT_POSES > $trg_pose_file
		ls $CLOUDS_DIR/*.pcd | sort | get_batch $(($src_i*$sensors_cnt)) $(($CUMULATED_FRAMES*$sensors_cnt)) > $src_clouds
		ls $CLOUDS_DIR/*.pcd | sort | get_batch $(($trg_i*$sensors_cnt)) $(($CUMULATED_FRAMES*$sensors_cnt)) > $trg_clouds

		echo -n "$src_i $trg_i "
		$BUT_VELODYNE_LIB/bin/cls-reg-subsequences --source_clouds_list $src_clouds --source_poses_file $src_pose_file --target_clouds_list $trg_clouds --target_poses_file $trg_pose_file -c $CALIBRATION -g 2 -p 1 --max_time_for_registration 100 --max_iterations 1000 --target_error 0.001
	fi
done > pairs.registered

$BUT_VELODYNE_LIB/scripts/poses_pairs_to_graph.py --poses $INIT_POSES --corrections pairs.registered | cat $POSE_GRAPH - > whole.graph
~/apps/SLAM_plus_plus_v2.10/bin/slam_plus_plus -i whole.graph
$BUT_VELODYNE_LIB/bin/slampp-solution-to-poses < solution.txt > solution.txt.poses
$BUT_VELODYNE_LIB/bin/build-3d-model -p solution.txt.poses -s1.0 -o solution.txt.poses.pcd --sensor_poses $CALIBRATION $(ls $CLOUDS_DIR/*.pcd | sort)
pdal translate solution.txt.poses.pcd solution.txt.poses.laz

rm $src_clouds $trg_clouds $src_pose_file $trg_pose_file
