#! /bin/bash

DATA_DIR=/media/files/geodrom/FIT_test/kancl/sequences/uhel_0/velodyne
OUT_DIR=/media/files/geodrom/FIT_test/kancl/sequences/uhel_0_fixed_distortion/velodyne
POSES=/media/files/geodrom/results/2-cls-kancl01/close-loop-uhel_0/solution-poses.txt
OFFSET=94

clouds_list=$(mktemp)
find $data_seq -name '*.pcd' -o -name '*.bin' | sort -g > $clouds_list

mkdir -p $OUT_DIR

current_poses=$(mktemp)
for i in $(seq $(($(wc -l < $clouds_list)-1)))
do
	head -n $(($i+1)) $POSES | tail -n2 > $current_poses
	cloud=$(head -n $i $clouds_list | tail -n1)
	~/workspace/but_velodyne_lib/bin/correct-velodyne-distortion $cloud $current_poses 94 $OUT_DIR/$i.pcd
done

rm $clouds_list $current_poses
