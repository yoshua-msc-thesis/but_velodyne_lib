#! /usr/bin/env python

import sys
import h5py
import numpy
import cv

from cv_yaml import load

class FeatureDB:
    def __init__(self):
        self.matrices = {}
    def get(self, filename, idx):
        if not filename in self.matrices:
            self.matrices[filename] = load(filename, "raw_gestalt_descriptors")
        return self.matrices[filename][idx, :]

db = FeatureDB()

for line in sys.stdin.readlines():
    tokens = line.split()
    src_feat = db.get(tokens[0], int(tokens[1]))
    trg_feat = db.get(tokens[2], int(tokens[3]))
    for feat in (src_feat, trg_feat):
        for i in range(feat.shape[0]):
            sys.stdout.write("%s "%feat[i])
    print ""
