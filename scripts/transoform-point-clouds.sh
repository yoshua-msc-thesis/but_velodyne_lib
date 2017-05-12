#! /bin/bash

if [ $# -ne 3 ]; then
	echo "ERROR, usage: $0 <clouds-list> <pose-file> <out-dir>"
	exit 1
fi

CLOUDS_LIST=$1
POSES_FILE=$2
OUTPUT_DIR=$3

mkdir -p $OUTPUT_DIR

paste -d" " $CLOUDS_LIST $POSES_FILE | while read line
do
	cloud=$(echo $line | cut -f1 -d" ")
	echo $line | cut -f2- -d" " > pose.tmp

	~/workspace/but_velodyne_lib/bin/transform-pcd -i $cloud -p pose.tmp -o $OUTPUT_DIR/$(basename $cloud)
done
