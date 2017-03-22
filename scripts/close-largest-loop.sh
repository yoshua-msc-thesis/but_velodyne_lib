#! /bin/bash

if [ $# -lt 3 ]
then
	echo "ERROR, usage: $0 [pose-file.txt] [sequence-dir] [output-dir]" >&2
	exit 1
fi

poses=$1
data_seq=$2
working_dir=$3

BIN_DIR=~/workspace/but_velodyne_lib/bin
SCRIPTS_DIR=~/workspace/but_velodyne_lib/scripts
SLAMPP=~/apps/SLAM_plus_plus_v2.10/bin/slam_plus_plus

mkdir -p $working_dir

first_cloud=$(find $data_seq -name '*.pcd' -o -name '*.bin' | sort -g | head -n1)
last_cloud=$(find $data_seq -name '*.pcd' -o -name '*.bin' | sort -g | tail -n1)
pcl_viewer $first_cloud $last_cloud -ax 1

$BIN_DIR/collar-lines-odom $first_cloud $last_cloud > $working_dir/loop.poses

$BIN_DIR/kitti-viewer -p $working_dir/loop.poses $first_cloud $last_cloud

$SCRIPTS_DIR/poses_to_graph.py < $poses > $working_dir/poses.graph
$SCRIPTS_DIR/poses_to_graph.py < $working_dir/loop.poses > $working_dir/loop.graph

graph_edges=$(wc -l < $working_dir/poses.graph)

sed "s/^EDGE3 0 1/EDGE3 0 $graph_edges/" $working_dir/loop.graph | cat $working_dir/poses.graph - > $working_dir/closed.graph

pushd $working_dir
	$SLAMPP -i closed.graph --pose-only --no-detailed-timing
	$BIN_DIR/slampp-solution-to-poses < solution.txt > closed.poses
popd

echo "Output saved to file: $working_dir/closed.poses"
#FIT_test/kancl/sequences/uhel_0/velodyne/1489496953200767.pcd
#FIT_test/kancl/sequences/uhel_0/velodyne/1489497259184684.pcd