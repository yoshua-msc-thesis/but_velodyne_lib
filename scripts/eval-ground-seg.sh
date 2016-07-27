#! /bin/bash

POINTS=100
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

if [ $# -ne 3 ] && [ $# -ne 4 ]; then
	echo "Expected arguments: <out-files-list> <ann-files-list> <output-file> [point-mask-files-list]" >&2
	exit 1
fi

pr_file=$3
rm -f $pr_file
touch $pr_file

out_ann=$(mktemp)

paste -d" " $1 $2 $4 | while read file_pair; do
	out_file=$(echo $file_pair | cut -f1 -d" ")
	ann_file=$(echo $file_pair | cut -f2 -d" ")
	ann_count=$(($(wc -l < $ann_file) - 1))
	ann_file_head=$(mktemp)
	head -n $ann_count $ann_file > $ann_file_head
	if [ "x$4" != "x" ]; then
		mask_file=$(echo $file_pair | cut -f3 -d" ")
		mask_file_head=$(mktemp)
		head -n $ann_count $mask_file > $mask_file_head
		head -n $ann_count $out_file | paste - $ann_file_head $mask_file_head | grep "1$" | cut -f1,2
		rm $mask_file_head
	else
		head -n $ann_count $out_file | paste - $ann_file_head
	fi
	rm $ann_file_head
done | sort -g | grep -v '^-1' > $out_ann


out_points_count=$(cat $(cat $1 | xargs) | wc -l)
ann_points_count=$(cat $(cat $2 | xargs) | wc -l)
ann_files_count=$(cat $2 | wc -l)
echo "Skipped $(($out_points_count - $ann_points_count + $ann_files_count)) points due to missing annotation" | tee -a $pr_file
echo "Skipped $(cat $(cat $1 | xargs) | grep '^-1' | wc -l) points due to missing output" | tee -a $pr_file
echo "Remaining $(wc -l < $out_ann) points" | tee -a $pr_file

echo threshold    TP   TN   FP   FN   precision   recall   fmeassure >>$pr_file
# for threshold = 0
echo -n "@ 0 " >>$pr_file
$SCRIPT_DIR/get_pr.py $out_ann 0 >>$pr_file

POINTS=$(($POINTS-1))
samples=$(wc -l < $out_ann)
for i in $(seq $POINTS); do
	th_line=$(($samples / $POINTS * $i))
	th=$(head -n $th_line $out_ann | tail -n 1 | cut -f1)
	echo -n "@ $th "
	$SCRIPT_DIR/get_pr.py $out_ann $th
done >>$pr_file
rm $out_ann

echo -n "average precision: " | tee -a $pr_file
grep "^@" $pr_file | tac | cut -f7,8 -d" " | $SCRIPT_DIR/compute_average_precision.py | tee -a $pr_file
