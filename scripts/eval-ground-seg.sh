#! /bin/bash

POINTS=10
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

if [ $# -ne 3 ]; then
	echo "Expected arguments: <out-files-pattern> <ann-files-pattern> <output-file>" >&2
	exit 1
fi

all_out=$(mktemp)
all_ann=$(mktemp)
cat $1 > $all_out
cat $2 > $all_ann

pr_file=$3

out_ann=out_ann #$(mktemp)
paste $all_out $all_ann | sort -g > $out_ann
rm $all_out $all_ann

# TP   TN   FP   FN   precision   recall   fmeassure
# for threshold = 0
$SCRIPT_DIR/get_pr.py $out_ann 0 > $pr_file

POINTS=$(($POINTS-1))
samples=$(wc -l < $out_ann)
for i in $(seq $POINTS); do
	th_line=$(($samples / $POINTS * $i))
	th=$(head -n $th_line $out_ann | tail -n 1 | cut -f1)
	$SCRIPT_DIR/get_pr.py $out_ann $th
done >>$pr_file
#rm $out_ann

tac $pr_file > tmp; mv tmp $pr_file

cut -f5,6 -d" " $pr_file | $SCRIPT_DIR/compute_average_precision.py $out_ann $th

