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

if [ $# -ne 5 ]; then
	echo "ERROR, expected usage: $0 <pcd-dir> <original-poses.txt> <input-pose.graph> <output-dir> <calibration.poses>" >&2
	exit 1
fi

PCD_DIR=$1
ORIGINAL_POSE_FILE=$2
INPUT_POSE_GRAPH=$3
OUTPUT_DIR=$4
CALIBRATION=$5

poses_cnt=$(wc -l < $ORIGINAL_POSE_FILE)

mkdir -p $OUTPUT_DIR
cp $INPUT_POSE_GRAPH $OUTPUT_DIR/04-subseq.graph

sensors_count=$(wc -l < $CALIBRATION)

new_poses=$OUTPUT_DIR/new-poses.txt
slampp $INPUT_POSE_GRAPH | head -n $poses_cnt > $new_poses

$BUT_BINS/pick-subsequences -p $new_poses | tee $OUTPUT_DIR/subsequences.txt

src_subseq_idx=0
src_poses=$OUTPUT_DIR/subseq.$src_subseq_idx.poses
src_clouds=$OUTPUT_DIR/subseq.$src_subseq_idx.clouds
from_to=$($BUT_SCRIPTS/get-subsequences.sh $OUTPUT_DIR/subsequences.txt $src_subseq_idx $ORIGINAL_POSE_FILE $PCD_DIR $sensors_count $src_poses $src_clouds)
src_idx_from=$(echo $from_to | cut -d" " -f1)
src_idx_to=$(echo $from_to | cut -d" " -f2)

trg_subseq_idx=1
trg_poses=$OUTPUT_DIR/subseq.$trg_subseq_idx.poses
trg_clouds=$OUTPUT_DIR/subseq.$trg_subseq_idx.clouds
from_to=$($BUT_SCRIPTS/get-subsequences.sh $OUTPUT_DIR/subsequences.txt $trg_subseq_idx $ORIGINAL_POSE_FILE $PCD_DIR $sensors_count $trg_poses $trg_clouds)
trg_idx_from=$(echo $from_to | cut -d" " -f1)
trg_idx_to=$(echo $from_to | cut -d" " -f2)

loop_transform=$OUTPUT_DIR/loop-seq-$src_subseq_idx-to-$trg_subseq_idx.txt
$BUT_BINS/cls-reg-subsequences --manual --source_clouds_list $src_clouds --source_poses_file $src_poses --target_clouds_list $trg_clouds --target_poses_file $trg_poses -c $CALIBRATION --max_time_for_registration 30 --max_iterations 500 -g 2 -p 1 --matching_threshold NO_THRESHOLD --target_error 0.001 | tee $loop_transform

$BUT_SCRIPTS/pose_to_edge.py --src_index_from $src_idx_from --src_index_to $src_idx_to --trg_index_from $trg_idx_from --trg_index_to $trg_idx_to -p $ORIGINAL_POSE_FILE -r $loop_transform -g $OUTPUT_DIR/04-subseq.graph | tee $OUTPUT_DIR/loop.edge
cat $OUTPUT_DIR/loop.edge >>$OUTPUT_DIR/04-subseq.graph

slampp $OUTPUT_DIR/04-subseq.graph | head -n $poses_cnt > $OUTPUT_DIR/04-subseq-closed.txt

$BUT_BINS/build-3d-model -p $OUTPUT_DIR/04-subseq-closed.txt $(ls $PCD_DIR/*.pcd | sort) -o $OUTPUT_DIR/04-subseq-closed.pcd --sensor_poses=$CALIBRATION -s 0.01
pdal translate $OUTPUT_DIR/04-subseq-closed.pcd $OUTPUT_DIR/04-subseq-closed.laz

cloudCompare $OUTPUT_DIR/04-subseq-closed.laz
