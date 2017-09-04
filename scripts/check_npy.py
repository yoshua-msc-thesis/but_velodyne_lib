#! /usr/bin/env python

import numpy as np
import sys

for fn in sys.argv[1:]:
    data = np.load(fn)
    print fn, data.shape
