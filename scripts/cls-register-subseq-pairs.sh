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

if [ $# -ne 7 ]
then
	echo "ERROR, expected arguments: $0 <clouds-dir> <init-poses> <original-pose-graph> <overlaps> <cumulated-frames> <calibration> <out-dir>" >&2
	exit 1
fi

CLOUDS_DIR=$1
INIT_POSES=$2
POSE_GRAPH=$3
OVERLAPS=$4
CUMULATED_FRAMES=$5
CALIBRATION=$6
OUT_DIR=$7

sensors_cnt=$(wc -l < $CALIBRATION)
threshold=$(awk '($1-1) == $2' $OVERLAPS | cut -d" " -f3 | sort -g | tail -n1)
threads=$(($(nproc)-1))

overlaps_filtered=$OUT_DIR/overlaps.txt.filtered
awk "\$5 < $threshold" $OVERLAPS > $overlaps_filtered

echo "Processing $(wc -l < $overlaps_filtered) frame-pairs in $threads processes in parallel" >&2

for id in $(seq $threads); do
	register_overlaping_frames $id $threads $overlaps_filtered > $OUT_DIR/pairs.registered.$id 2> $OUT_DIR/pairs.registration.$id.log &
done

wait

for id in $(seq $threads); do
	$BUT_VELODYNE_LIB/scripts/poses_pairs_to_graph.py --poses $INIT_POSES --corrections $OUT_DIR/pairs.registered.$id
done | cat $POSE_GRAPH - > $OUT_DIR/with-overlaps.graph

pushd $OUT_DIR
	~/apps/SLAM_plus_plus_v2.10/bin/slam_plus_plus -i with-overlaps.graph
popd
$BUT_VELODYNE_LIB/bin/slampp-solution-to-poses < $OUT_DIR/solution.txt > $OUT_DIR/overlaps-aligned.poses
$BUT_VELODYNE_LIB/bin/build-3d-model -p $OUT_DIR/overlaps-aligned.poses -s1.0 -o $OUT_DIR/overlaps-aligned.pcd --sensor_poses $CALIBRATION $(ls $CLOUDS_DIR/*.pcd | sort)
pdal translate $OUT_DIR/overlaps-aligned.pcd $OUT_DIR/overlaps-aligned.laz
