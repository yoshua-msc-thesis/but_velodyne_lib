#! /usr/bin/env python

import sys

if len(sys.argv) != 3:
    sys.stderr.write("Expected arguments: <out-ann-2cols-file> <threshold>\n")
    sys.exit(1)
    
out_ann_file = open(sys.argv[1])
threshold = float(sys.argv[2])

tp = tn = fp = fn = 0.0

for line in out_ann_file.readlines():
    (out, ann) = line.split()
    out = float(out)
    ann = int(ann)

    if ann == 1:
        if out >= threshold:
            tp += 1
        else:
            fn += 1
    else:
        if out >= threshold:
            fp += 1
        else:
            tn += 1

precision = tp / (tp + fp)
recall = tp / (tp + fn)

if precision+recall > 0:
    fmeassure = 2 * precision * recall / (precision + recall)
else:
    fmeassure = "NaN"

print tp, tn, fp, fn, precision, recall, fmeassure
