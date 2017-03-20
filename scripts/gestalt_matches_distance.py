#! /usr/bin/env python

import sys
import h5py
import numpy as np
import cv

from cv_yaml import load

class FeatureDB:
    def __init__(self):
        self.matrices = {}
    def get(self, filename, idx):
        if not filename in self.matrices:
            self.matrices[filename] = load(filename, "raw_gestalt_descriptors")
        return self.matrices[filename][idx, :]

def load_matrix(filename):
    f = open(filename)
    rows = [np.array(map(float, line.split())) for line in f.readlines()]
    return np.array(rows)

def remap_by_polynom(features, polynoms):
    deg = polynoms.shape[1]
    feat_powers = [np.ones(features.shape)]
    for _ in range(1, deg):
        feat_powers = [np.multiply(feat_powers[0], features)] + feat_powers
    feat_powers_mat = np.array(feat_powers)
    return np.einsum("ij,ij->i", np.transpose(feat_powers_mat), polynoms)

polynoms = None
A = None
if len(sys.argv) > 1:
    polynoms = load_matrix(sys.argv[1])     # 66*6
    if len(sys.argv) > 2:
        A = load_matrix(sys.argv[2])        # 66*66

db = FeatureDB()

for line in sys.stdin.readlines():
    tokens = line.split()
    src_feat = db.get(tokens[0], int(tokens[1]))
    trg_feat = db.get(tokens[2], int(tokens[3]))

    if polynoms is not None:
        src_feat = remap_by_polynom(src_feat, polynoms)
        trg_feat = remap_by_polynom(trg_feat, polynoms)
    if A is not None:
        src_feat = np.dot(src_feat, A)
        trg_feat = np.dot(trg_feat, A)

    print np.linalg.norm(src_feat-trg_feat)
