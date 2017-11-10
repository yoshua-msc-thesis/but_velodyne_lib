#! /bin/bash

if [ $# -ne 6 ]
then
	echo "ERROR! Expected arguments: <in-dir> <out-dir> <init-poses> <pose-graph> <calibration> <clouds-dir>" >&2
	exit 1
fi

in_dir=$1
out_dir=$2
init_poses=$3
pose_graph=$4
calibration=$5
clouds_dir=$6

for file in $in_dir/pairs.registration.*.log
do
	i=$(echo $file | rev | cut -d'.' -f2 | rev)

	grep max -B4 $file | grep -o '[0-9]\+.1.pcd' | cut -d"." -f1 | sed 's/^0\(.\)/\1/' | pr -c2 -a -ts" " | awk '{printf("%d %d\n", $1, $2)}' > $out_dir/unconverged.$i.maxterm

	cp $in_dir/pairs.registered.$i $out_dir/pairs.registered.$i.converged
	while read pair
	do
		grep -v "^$pair " $out_dir/pairs.registered.$i.converged > tmp
		mv tmp $out_dir/pairs.registered.$i.converged
	done < $out_dir/unconverged.$i.maxterm

	$BUT_VELODYNE_LIB/scripts/poses_pairs_to_graph.py --poses $init_poses --corrections $out_dir/pairs.registered.$i.converged --cumulated_frames 1
done | cat $pose_graph - > $out_dir/with-overlaps.graph

pushd $out_dir
	~/apps/SLAM_plus_plus_v2.10/bin/slam_plus_plus -i with-overlaps.graph
popd
$BUT_VELODYNE_LIB/bin/slampp-solution-to-poses < $out_dir/solution.txt > $out_dir/overlaps-converted-aligned.poses
$BUT_VELODYNE_LIB/bin/build-3d-model -p $out_dir/overlaps-converted-aligned.poses -s0.33 -o $out_dir/overlaps-converted-aligned.pcd --sensor_poses $calibration $(ls $clouds_dir/*.pcd | sort)
pdal translate $out_dir/overlaps-converted-aligned.pcd $out_dir/overlaps-converted-aligned.laz
