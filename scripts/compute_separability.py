#! /usr/bin/env python

import sys, math

BATCH_SIZE=20000

sums = [0]
thresholds = [0]
zeros = ones = 0
for i, line in enumerate(sys.stdin.readlines()):
    tokens = line.split()
    prediction = float(tokens[0])
    gt = float(tokens[1])
    if i%BATCH_SIZE == 0:
        sums.append(0)
        thresholds.append(prediction)
    sums[-1] += gt
    thresholds[-1] = prediction
    zeros += abs(gt-1.0)
    ones += gt

assert len(sums) == len(thresholds)

for i,th in enumerate(thresholds):
    Pmiss = sum(sums[i:]) / ones
    Pfa = (min(BATCH_SIZE*(i+1), all) - sum(sums[0:i+1])) / zeros
    sys.stdout.write(("%s;"*4 + "\n")%(th, Pmiss, Pfa, (1-Pfa)*(1-Pmiss)))
