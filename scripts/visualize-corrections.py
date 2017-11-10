#! /usr/bin/env python

import sys
import numpy as np
from odometry_cnn_data import load_poses_corrections
import plotly.plotly as py
import plotly.graph_objs as go

REF_PT = np.array([[10], [10], [10], [1]])

corrections = load_poses_corrections(sys.stdin)
poses_cout = max([max(c["src_i"],c["trg_i"]) for c in corrections])+1

heatmap = [[0]*poses_cout for _ in range(poses_cout)]

for c in corrections:
    new_pt = c["pose"].M * REF_PT
    dist = np.linalg.norm(new_pt-REF_PT)
    heatmap[c["src_i"]][c["trg_i"]] = dist
    print dist

trace = go.Heatmap(z=heatmap)
data=[trace]
py.iplot(data, filename='dist-heatmap')
