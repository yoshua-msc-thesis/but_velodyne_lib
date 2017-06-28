#! /bin/bash

if [ $# -ne 7 ]; then
	echo "ERROR, usage: $0 <subsequences-list.txt> <subseq-idx> <poses.txt> <clouds-dir> <sensors_count> <out-poses.txt> <out-clouds.list>" >&2
	exit 1
fi

src_idx=$(head -n $(($2+1)) $1 | tail -n1 | cut -d" " -f1)
trg_idx=$(head -n $(($2+1)) $1 | tail -n1 | cut -d" " -f2)

count=$(($trg_idx-$src_idx+1))

head -n $(($trg_idx+1)) $3 | tail -n $count > $6

ls $4/*.pcd | sort | head -n $((($trg_idx+1)*$5)) | tail -n $(($count*$5)) > $7

echo "$src_idx $trg_idx"
