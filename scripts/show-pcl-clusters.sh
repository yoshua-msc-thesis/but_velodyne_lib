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

if [ $# -ne 4 ]; then
	echo "ERROR, expected usage: $0 <pcd-dir> <pose-file.txt> <calibration.poses> <subsequences.txt>" >&2
	exit 1
fi

PCD_DIR=$1
POSE_FILE=$2
CALIBRATION=$3
SUBSEQUENCES=$4

sensors_count=$(wc -l < $CALIBRATION)
clusters_count=$(wc -l < $SUBSEQUENCES)

for idx in $(seq 0 $(($clusters_count-1)))
do
	poses=/tmp/cluster.poses
	clouds=/tmp/cluster.clouds
	from_to=$($BUT_SCRIPTS/get-subsequences.sh $SUBSEQUENCES $idx $POSE_FILE $PCD_DIR $sensors_count $poses $clouds)
	idx_from=$(echo $from_to | cut -d" " -f1)
	idx_to=$(echo $from_to | cut -d" " -f2)

	$BUT_BINS/build-3d-model -p $poses $(cat $clouds) -o /tmp/subseq-$idx.pcd --sensor_poses=$CALIBRATION
	clusters="/tmp/subseq-$idx.pcd $clusters"
done

pcl_viewer $clusters
