#! /bin/bash

SLAMPP=~/apps/SLAM_plus_plus_v2.10/bin/slam_plus_plus

function slampp {
	in=$1
	pushd $(dirname $in) >/dev/null
		$SLAMPP -i $(basename $in) --pose-only --no-detailed-timing --silent >&2
	popd >/dev/null
	$BUT_VELODYNE_LIB/bin/slampp-solution-to-poses < $(dirname $in)/solution.txt
}

if [ $# -ne 5 ]
then
	echo "ERROR, usage: $0 <clouds.list> <unclosed-poses.txt> <graph> <out-dir> <sensor-calibration>" >&2
	exit 1
fi

clouds=$1
poses=$2
out_dir=$4
calibration=$5

pose_graph=$out_dir/pose.graph
cp $3 $pose_graph
frames_num=$(wc -l < $poses)

original_poses=$poses
src_clouds=$out_dir/src_clouds.txt
trg_clouds=$out_dir/trg_clouds.txt
src_poses=$out_dir/src_poses.txt
trg_poses=$out_dir/trg_poses.txt
out_transform=$out_dir/transform-sequences.txt

expected_poses_cnt=$(wc -l < $original_poses)

init_N=4
N=$init_N
direction="up"
max_N=$(($N/2*3))
for times in $(seq 128); do
	for end1 in $(seq $N $N $(($frames_num-$N/2))); do
		start1=$(($end1-$N))
		start2=$(($start1+$N/2))
		end2=$(($end1+$N/2))

		head -n $(($end1*2)) $clouds | tail -n $(($N*2)) | tee $src_clouds
		head -n $(($end2*2)) $clouds | tail -n $(($N*2)) | tee $trg_clouds
		head -n $end1 $poses | tail -n $N > $src_poses
		head -n $end2 $poses | tail -n $N > $trg_poses

		$BUT_VELODYNE_LIB/bin/cls-reg-subsequences --source_clouds_list $src_clouds --source_poses_file $src_poses --target_clouds_list $trg_clouds --target_poses_file $trg_poses -g 2 -p 1 -c $calibration --target_error 0.001 --max_time_for_registration 30 --max_iterations 1000 | tee $out_transform

		$BUT_VELODYNE_LIB/scripts/pose_to_edge.py --src_index_from $start1 --src_index_to $(($end1-1)) --trg_index_from $start2 --trg_index_to $(($end2-1)) -p $poses -o $original_poses -r $out_transform -g $pose_graph | tee $out_dir/loop.edges
		cat $out_dir/loop.edges >>$pose_graph

		echo "================================================================================"
	done
	poses=$out_dir/poses-times-$times-N-$N.txt
	slampp $pose_graph | head -n $expected_poses_cnt > $poses
	$BUT_VELODYNE_LIB/bin/build-3d-model -p $poses $(cat $clouds) -o $poses.pcd --sensor_poses $calibration
	
	if [ $direction == "up" ]; then
		N=$(($N*2))
		if [ $(($N/2*3)) -gt $frames_num ]; then
			direction="down"
			N=$(($N/4))
		fi
	elif [ $direction == "down" ]; then
		N=$(($N/2))
		if [ $N -lt $init_N ]; then
			direction="up"
			N=$(($N*4))
		fi
	fi
done
