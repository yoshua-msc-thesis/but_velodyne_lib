#! /bin/bash

if [ $# -ne 6 ]; then
	echo "ERROR, usage: $0 <subsequences-list.txt> <subseq-idx> <poses.txt> <clouds-dir> <out-poses.txt> <out-clouds.list>" >&2
	exit 1
fi

src_idx=$(head -n $(($2+1)) $1 | tail -n1 | cut -d" " -f1)
trg_idx=$(head -n $(($2+1)) $1 | tail -n1 | cut -d" " -f2)
count=$(($trg_idx-$src_idx+1))

head -n $(($trg_idx+1)) $3 | tail -n $count > $5
ls $4/*.pcd | sort | head -n $(($trg_idx+1)) | tail -n $count > $6

echo "Poses and clouds from index $src_idx to $trg_idx were listed in $5 $6" >&2
