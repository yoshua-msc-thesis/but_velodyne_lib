#! /bin/bash

BUT_SCRIPTS=~/workspace/but_velodyne_lib/scripts
BUT_BINS=~/workspace/but_velodyne_lib/bin
SLAMPP=~/apps/SLAM_plus_plus_v2.10/bin/slam_plus_plus

function slampp {
	in=$1
	pushd $(dirname $in) >/dev/null
		$SLAMPP -i $(basename $in) --pose-only --no-detailed-timing --silent >&2
	popd >/dev/null
	$BUT_BINS/slampp-solution-to-poses < $(dirname $in)/solution.txt
}

if [ $# -ne 8 ]; then
	echo "ERROR, expected usage: $0 <pcd-dir> <pose-file.txt> <input-pose.graph> <calibration.poses> <output-dir> <start-idx> <end-idx> <length>" >&2
	exit 1
fi

PCD_DIR=$1
POSE_FILE=$2
INPUT_POSE_GRAPH=$3
CALIBRATION=$4
OUTPUT_DIR=$5
BREAK_START=$6
BREAK_END=$7
FRAMES_TO_USE=$8

expected_poses_cnt=$(wc -l < $POSE_FILE)

mkdir -p $OUTPUT_DIR
cp $INPUT_POSE_GRAPH $OUTPUT_DIR/pose.graph

sensors_count=$(wc -l < $CALIBRATION)

poses_from_graph=$INPUT_POSE_GRAPH.poses.txt
slampp $INPUT_POSE_GRAPH | head -n $expected_poses_cnt > $poses_from_graph

src_poses=$OUTPUT_DIR/src.poses
src_clouds=$OUTPUT_DIR/src.clouds
head -n$BREAK_START $POSE_FILE | tail -n$FRAMES_TO_USE > $src_poses
ls $PCD_DIR/*.pcd | sort | head -n$(($BREAK_START*2)) | tail -n$(($FRAMES_TO_USE*2)) > $src_clouds
src_idx_from=$(($BREAK_START-$FRAMES_TO_USE))
src_idx_to=$(($BREAK_START-1))

trg_poses=$OUTPUT_DIR/trg.poses
trg_clouds=$OUTPUT_DIR/trg.clouds
head -n$(($BREAK_END+$FRAMES_TO_USE)) $POSE_FILE | tail -n$FRAMES_TO_USE > $trg_poses
ls $PCD_DIR/*.pcd | sort | head -n$((($BREAK_END+$FRAMES_TO_USE)*2)) | tail -n$(($FRAMES_TO_USE*2)) > $trg_clouds
trg_idx_from=$BREAK_END
trg_idx_to=$(($BREAK_END+$FRAMES_TO_USE-1))

transform=$OUTPUT_DIR/reg-transform.txt
$BUT_BINS/cls-reg-subsequences --manual --source_clouds_list $src_clouds --source_poses_file $src_poses --target_clouds_list $trg_clouds --target_poses_file $trg_poses -c $CALIBRATION --max_time_for_registration 30 --max_iterations 1000 -g 2 -p 1 | tee $transform

$BUT_SCRIPTS/pose_to_edge.py --src_index_from $src_idx_from --src_index_to $src_idx_to --trg_index_from $trg_idx_from --trg_index_to $trg_idx_to -o $POSE_FILE -p $poses_from_graph -r $transform -g $INPUT_POSE_GRAPH | tee $OUTPUT_DIR/loop.edge
cat $INPUT_POSE_GRAPH $OUTPUT_DIR/loop.edge > $OUTPUT_DIR/pose.graph

slampp $OUTPUT_DIR/pose.graph | head -n $expected_poses_cnt > $OUTPUT_DIR/break-closed.txt

$BUT_BINS/build-3d-model -p $OUTPUT_DIR/break-closed.txt $(ls $PCD_DIR/*.pcd | sort) -o $OUTPUT_DIR/break-closed.pcd --sensor_poses=$CALIBRATION
pcl_viewer $OUTPUT_DIR/break-closed.pcd.rgb.pcd
pcl_viewer $OUTPUT_DIR/break-closed.pcd.poses.pcd
