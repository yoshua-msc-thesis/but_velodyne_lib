#! /usr/bin/env python

# ~/workspace/but_velodyne_lib/scripts/poses_to_graph.py < 03-poses-cls-m10.txt > 04-cls-subseq.unclosed.graph; ~/workspace/but_velodyne_lib/scripts/pose_to_edge.py --src_index_from 253 --src_index_to 495 --trg_index_from 1849 --trg_index_to 2120 -p 03-poses-cls-m10.txt -r forward.1-to-backward.9.poses -g 04-cls-subseq.unclosed.graph >>04-cls-subseq.unclosed.graph; cat loop.txt >>04-cls-subseq.unclosed.graph; ~/apps/SLAM_plus_plus_v2.10/bin/slam_plus_plus -i 04-cls-subseq.unclosed.graph --pose-only --no-detailed-timing; ~/workspace/but_velodyne_lib/bin/slampp-solution-to-poses < solution.txt > 04-cls-subseq.closed.txt; ~/workspace/but_velodyne_lib/bin/build-3d-model -p 04-cls-subseq.closed.txt $(ls fixed-by-03-poses-cls-m10/*.pcd | sort) -o 04-cls-subseq.closed.pcd; pcl_viewer 04-cls-subseq.closed.pcd.rgb.pcd 

import sys
import argparse

from odometry_cnn_data import Odometry, Edge3D, load_kitti_poses, get_delta_odometry

class EdgesGenerator:
    def __init__(self, new_poses, original_poses, reg_pose, graph_file):
        self.new_poses = new_poses
        self.original_poses = original_poses
        self.reg_pose = reg_pose
        self.max_vertex = self.getMaxVertex(graph_file)

    def getMaxVertex(self, graph_file):
        max_vertex = -1
        for line in open(graph_file).readlines():
            tokens = line.split()
            src_vertex = int(tokens[1])
            trg_vertex = int(tokens[2])
            max_vertex = max(max_vertex, max(src_vertex, trg_vertex))
        return max_vertex

    def genEdgesToNewVertex(self, idx_from, idx_to):
        self.max_vertex = new_vertex = self.max_vertex+1
        edges = []
        for i in range(idx_from, idx_to+1):
            t = self.original_poses[i].inv() * self.new_poses[0]
            edges.append(Edge3D(i, new_vertex, t.dof))
        return edges

    def genEdges(self, src_idx_from, src_idx_to, trg_idx_from, trg_idx_to):
        edges  = self.genEdgesToNewVertex(src_idx_from, src_idx_to)
        edges += self.genEdgesToNewVertex(trg_idx_from, trg_idx_to)
        edges.append(Edge3D(self.max_vertex-1, self.max_vertex, self.reg_pose.dof))
        return edges

############ obsolete: ############
    def getCorrection(self, src_index, trg_index):
        src_new = self.new_poses[src_index]
        src_old = self.original_poses[src_index]
        trg_new = self.new_poses[trg_index]
        trg_old = self.original_poses[trg_index]
        return trg_new.inv() * trg_old * self.reg_pose * src_old.inv() * src_new

    def printEdge(self, src_index, trg_index):
        src_pose = self.original_poses[src_index]
        trg_pose = self.original_poses[trg_index]
        t = src_pose.inv() * self.reg_pose * trg_pose
        print Edge3D(src_index, trg_index, t.dof)

    def printEdges(self, src_index_from, src_index_to, trg_index_from, trg_index_to):
        if src_index_from <= src_index_to and trg_index_from <= trg_index_to:
            src_middle_index = (src_index_from + src_index_to) / 2
            trg_middle_index = (trg_index_from + trg_index_to) / 2

            self.printEdges(src_index_from, src_middle_index-1, trg_index_from, trg_middle_index-1)
            self.printEdge(src_middle_index, trg_middle_index)
            self.printEdges(src_middle_index+1, src_index_to, trg_middle_index+1, trg_index_to)

    def printEdgesToNewVertex(self, poses, index_from, index_to, new_vertex):
        start_pose = poses[index_from]
        for i in range(index_from, index_to+1, 50):
            t = start_pose.inv() * poses[i]
            print Edge3D(new_vertex, i, t.dof)
############ obsolete. ############

parser = argparse.ArgumentParser(description="Pose from subsequence registration to EDGE3D")
parser.add_argument("--src_index_from", dest="src_index_from", type=int, required=True)
parser.add_argument("--src_index_to", dest="src_index_to", type=int, required=True)
parser.add_argument("--trg_index_from", dest="trg_index_from", type=int, required=True)
parser.add_argument("--trg_index_to", dest="trg_index_to", type=int, required=True)
parser.add_argument("-p", "--poses", dest="poses", type=str, required=True)
parser.add_argument("-o", "--original_poses", dest="original_poses", type=str, required=False)
parser.add_argument("-r", "--registration_pose", dest="registration_pose", type=str, required=True)
parser.add_argument("-g", "--graph_file", dest="graph_file", type=str, required=True)
args = parser.parse_args()

reg_poses = load_kitti_poses(args.registration_pose)
assert len(reg_poses) == 1
reg_pose = reg_poses[0]
poses = load_kitti_poses(args.poses)

if hasattr(args, "original_poses") and args.original_poses:
    original_poses = load_kitti_poses(args.original_poses)
else:
    original_poses = poses

generator = EdgesGenerator(poses, original_poses, reg_pose, args.graph_file)
edges = generator.genEdges(args.src_index_from, args.src_index_to, args.trg_index_from, args.trg_index_to)
for e in edges:
    print e
