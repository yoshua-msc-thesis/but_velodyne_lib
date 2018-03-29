#! /usr/bin/env python

import sys

quaternions = {}
accelerations = {}

for line in sys.stdin.readlines():
    tokens = line.split()
    time = float(tokens[1])
    if tokens[0] == "q":
        quaternions[time] = tokens[2:]
    else:
        accelerations[time] = tokens[2:]

last_time = -1
for kq in sorted(quaternions):
    ka = min(accelerations, key=lambda ka: abs(kq - ka))
    if last_time > 0:
        print kq-last_time, " ".join(accelerations[ka]), " ".join(quaternions[ka])
    last_time = kq
