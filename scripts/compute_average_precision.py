#! /usr/bin/env python

import sys

last_prec = last_recall = -1.0
average_prec = 0.0

for line in sys.stdin.readlines():
    (prec, recall) = map(float, line.split())

    if last_prec > 0 and last_recall > 0:
        average_prec += (last_prec + prec) / 2 * (recall - last_recall)
    
    last_prec = prec
    last_recall = recall

print average_prec
