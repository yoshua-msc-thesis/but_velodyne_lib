#! /bin/bash

#set -o xtrace

if [ $# -lt 4 ]
then
	echo "ERROR, usage: $0 [pose-file.txt] [pose.graph] [sequence-dir] [output-dir]" >&2
	exit 1
fi

poses=$1
pose_graph=$2
data_seq=$3
working_dir=$4

BIN_DIR=~/workspace/but_velodyne_lib/bin
SCRIPTS_DIR=~/workspace/but_velodyne_lib/scripts
SLAMPP=~/apps/SLAM_plus_plus_v2.10/bin/slam_plus_plus

mkdir -p $working_dir

$SCRIPTS_DIR/find_random_loop.py < $poses > $working_dir/loop.tmp
src_i=$(head -n1 $working_dir/loop.tmp | awk '{print $1}')
trg_i=$(head -n1 $working_dir/loop.tmp | awk '{print $2}')
tail $working_dir/loop.tmp -n+2 > $working_dir/loop-init.poses
echo "Closing loop: $src_i $trg_i in $working_dir" >&2

src_cloud=$(find $data_seq -name '*.pcd' -o -name '*.bin' | sort -g | head -n $(($src_i+1)) | tail -n1)
trg_cloud=$(find $data_seq -name '*.pcd' -o -name '*.bin' | sort -g | head -n $(($trg_i+1)) | tail -n1)
#pcl_viewer $src_cloud $trg_cloud -ax 1

#$BIN_DIR/kitti-viewer -p $working_dir/loop-init.poses $src_cloud $trg_cloud

$BIN_DIR/manual-registration --src_cloud $src_cloud --trg_cloud $trg_cloud --pose_file $working_dir/loop-init.poses > $working_dir/loop.poses

#$BIN_DIR/kitti-viewer -p $working_dir/loop.poses $src_cloud $trg_cloud

$SCRIPTS_DIR/poses_to_graph.py < $working_dir/loop.poses > $working_dir/loop.graph

graph_edges=$(wc -l < $working_dir/poses.graph)

sed "s/^EDGE3 0 1/EDGE3 $src_i $trg_i/" $working_dir/loop.graph | cat $pose_graph - > $working_dir/closed.graph

pushd $working_dir
	$SLAMPP -i closed.graph --pose-only --no-detailed-timing
	$BIN_DIR/slampp-solution-to-poses < solution.txt > closed.poses
popd

echo "Output saved to file: $working_dir/closed.poses"
