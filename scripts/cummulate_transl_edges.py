#! /usr/bin/env python

import sys
from odometry_cnn_data import Odometry
import path

if len(sys.argv) != 3:
    sys.stderr.write("Invalid arguments, expected: <translation-to-cummulate.graph> <rotation.graph>\n")
    sys.exit(1)

#EDGE3 src_i trg_i 6*DoF 21*precission_matrix
class Edge:
    def __init__(self, src_i, trg_i, dof, precision):
        self.src_i = src_i
        self.trg_i = trg_i
        self.dof = dof
        self.precision = precision

    @classmethod
    def fromGraphLine(cls, line_of_graph_file):
        tokens = line_of_graph_file.strip().split()
        src_i, trg_i = map(int, tokens[1:3])
        dof = map(float, tokens[3:9])
        precision = map(float, tokens[9:])
        return cls(src_i, trg_i, dof, precision)

    def replace_rotation_by(self, other):
        assert self.src_i == other.src_i
        assert self.trg_i == other.trg_i
        self.dof[3:6] = other.dof[3:6]

    def replace_translation_by(self, other):
        assert self.src_i == other.src_i
        assert self.trg_i == other.trg_i
        self.dof[0:3] = other.dof[0:3]
    
    def __str__(self):
        return "EDGE3 " + " ".join(map(str, [self.src_i, self.trg_i] + self.dof + self.precision))
    
    def __repr__(self):
        return "<" + str(self) + ">"

    def __add__(self, other):
        assert self.trg_i == other.src_i
        new_odom = Odometry().move(self.dof).move(other.dof)
        return Edge(self.src_i, other.trg_i, new_odom.dof, self.precision)
    
    def __neg__(self):
        new_odom = Odometry().move(self.dof, True)
        return Edge(self.trg_i, self.src_i, new_odom.dof, self.precision)

    def __sub__(self, other):
        return self + (-other)

def find_connections_from(index, edges):
    return [e for e in edges if e.src_i == index]

def find_connections_to(index, edges):
    return [e for e in edges if e.trg_i == index]

# DFS by recursion
def find_path(src_i, trg_i, edges):
    if src_i < trg_i:
        for conn in find_connections_from(src_i, edges):
            result = [conn] + find_path(conn.trg_i, trg_i, edges)
            if result[-1].trg_i == trg_i:
                return result
    return []

def split_first(edges):
    for e in edges:
        if e.trg_i - e.src_i > 1:
            for c in find_connections_to(e.trg_i, edges):
                if c.src_i > e.src_i:
                    edges.append(e - c)
            for c in find_connections_from(e.src_i, edges):
                if c.trg_i < e.trg_i:
                    edges.append(-c + e)
    return edges

translations = map(Edge.fromGraphLine, open(sys.argv[1]).readlines())
translations = split_first(translations)

edges = map(Edge.fromGraphLine, open(sys.argv[2]).readlines())
for rot_edge in edges:
    path = find_path(rot_edge.src_i, rot_edge.trg_i, translations)
    for i in range(1, len(path)):
        path[0] = path[0] + path[i]
    rot_edge.replace_translation_by(path[0])
    print rot_edge
