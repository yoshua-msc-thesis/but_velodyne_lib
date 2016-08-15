#! /usr/bin/env python

import numpy as np
import cv

def load(yaml_filename, node_name):
    return np.asarray(cv.Load(yaml_filename, cv.CreateMemStorage(), node_name))
