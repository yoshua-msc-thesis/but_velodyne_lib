#! /bin/bash

if [ $# -ne 3 ]; then
	echo "ERROR, usage: $0 <poses.txt> <clouds-dir> <skip-file.txt>" >&2
	exit 1
fi

POSE_FILE=$1
CLOUDS_DIR=$2
SKIP_FILE=$3

ls $CLOUDS_DIR/*.pcd | sort > all_clouds.tmp
paste all_clouds.tmp $POSE_FILE -d" " | head -n $(wc -l < $POSE_FILE) | head -n $(wc -l < all_clouds.tmp) > cloud_and_poses.tmp

rm to_fix_lines.tmp
while read line
do
	grep $line cloud_and_poses.tmp >>to_fix_lines.tmp
	grep -v $line cloud_and_poses.tmp > tmp; mv tmp cloud_and_poses.tmp
done < $SKIP_FILE

cut -d" " -f1 < cloud_and_poses.tmp > map_clouds_list.tmp
cut -d" " -f2- < cloud_and_poses.tmp > map_poses.tmp

while read line
do
	echo $line | cut -d" " -f1 > to_fix_cloud.tmp
	echo $line | cut -d" " -f2- > to_fix_pose.tmp
	$BUT_VELODYNE_LIB/bin/cls-reg-subsequences --source_clouds_list map_clouds_list.tmp --source_poses_file map_poses.tmp\
	                                           --target_clouds_list to_fix_cloud.tmp --target_poses_file to_fix_pose.tmp\
	                                           --target_error 0.001 -g 10 -p 2 --max_time_for_registration 120
done < to_fix_lines.tmp
