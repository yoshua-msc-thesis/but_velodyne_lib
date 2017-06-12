#! /bin/bash

BUT_SCRIPTS=~/workspace/but_velodyne_lib/scripts
BUT_BINS=~/workspace/but_velodyne_lib/bin
SLAMPP=~/apps/SLAM_plus_plus_v2.10/bin/slam_plus_plus

function slampp {
	in=$1
	out=$2
	pushd $(dirname $in)
		$SLAMPP -i $(basename $in) --pose-only --no-detailed-timing --silent
	popd
	$BUT_BINS/slampp-solution-to-poses < $(dirname $in)/solution.txt > $out
}

if [ $# -ne 5 ]; then
	echo "ERROR, expected usage: $0 <pcd-dir> <pose-file.txt> <input-pose.graph> <subseq-count> <output-dir>" >&2
	exit 1
fi

PCD_DIR=$1
POSE_FILE=$2
INPUT_POSE_GRAPH=$3
CLUSTERS=$4
OUTPUT_DIR=$5

mkdir -p $OUTPUT_DIR
cp $INPUT_POSE_GRAPH $OUTPUT_DIR/04-subseq.graph

slampp $INPUT_POSE_GRAPH $OUTPUT_DIR/initial-poses.txt
echo "Clustering ..."
$BUT_BINS/cluster-subsequences -p $OUTPUT_DIR/initial-poses.txt -c $CLUSTERS $(ls $PCD_DIR/*.pcd | sort) | tee $OUTPUT_DIR/subsequences.txt

echo "Source index:"
read src_subseq_idx
src_poses=$OUTPUT_DIR/subseq.$src_subseq_idx.poses
src_clouds=$OUTPUT_DIR/subseq.$src_subseq_idx.clouds
from_to=$($BUT_SCRIPTS/get-subsequences.sh $OUTPUT_DIR/subsequences.txt $src_subseq_idx $POSE_FILE $PCD_DIR $src_poses $src_clouds)
src_idx_from=$(echo $from_to | cut -d" " -f1)
src_idx_to=$(echo $from_to | cut -d" " -f2)

echo "Target index:"
read trg_subseq_idx
trg_poses=$OUTPUT_DIR/subseq.$trg_subseq_idx.poses
trg_clouds=$OUTPUT_DIR/subseq.$trg_subseq_idx.clouds
from_to=$($BUT_SCRIPTS/get-subsequences.sh $OUTPUT_DIR/subsequences.txt $trg_subseq_idx $POSE_FILE $PCD_DIR $trg_poses $trg_clouds)
trg_idx_from=$(echo $from_to | cut -d" " -f1)
trg_idx_to=$(echo $from_to | cut -d" " -f2)

loop_transform=$OUTPUT_DIR/loop-seq-$src_subseq_idx-to-$trg_subseq_idx.txt
$BUT_BINS/cls-reg-subsequences --source_clouds_list $src_clouds --source_poses_file $src_poses --target_clouds_list $trg_clouds --target_poses_file $trg_poses --max_time_for_registration 30 --max_iterations 500 -g 2 -p 1 --matching_threshold NO_THRESHOLD | tee $loop_transform

$BUT_SCRIPTS/pose_to_edge.py --src_index_from $src_idx_from --src_index_to $src_idx_to --trg_index_from $trg_idx_from --trg_index_to $trg_idx_to -p $POSE_FILE -r $loop_transform -g $OUTPUT_DIR/04-subseq.graph >>$OUTPUT_DIR/04-subseq.graph

slampp $OUTPUT_DIR/04-subseq.graph $OUTPUT_DIR/04-subseq-closed.txt

$BUT_BINS/build-3d-model -p $OUTPUT_DIR/04-subseq-closed.txt $(ls $PCD_DIR/*.pcd | sort) -o $OUTPUT_DIR/04-subseq-closed.pcd
pcl_viewer $OUTPUT_DIR/04-subseq-closed.pcd.rgb.pcd
pcl_viewer $OUTPUT_DIR/04-subseq-closed.pcd.poses.pcd
