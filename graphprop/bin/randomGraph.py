#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2015 Philipp Robbel
# Based in part on graph generation example from graph-tool documentation.
# 
# License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.
# 
# The software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

"""Generate an (undirected) random graph with a specified number of nodes and an upper limit on the  out-degree of each node.

Run as ./randomGraph <nodes> <max_degree>
"""

from graph_tool.all import *
from numpy.random import *
import sys, os, os.path
import numpy as np

#seed(42)
#seed_rng(42)

# Rejection sampling for vertex degree
def sample_k(max):
    accept = False
    while not accept:
        k = randint(1,max+1)
        accept = random() < 1.0/k
    return k

if __name__ == "__main__":
  if len(sys.argv) != 3:
    print "Usage: python randomGraph.py <nodes> <max_degree>"
    sys.exit(0)

  g = random_graph(int(sys.argv[1]), lambda: sample_k(int(sys.argv[2])), model="uncorrelated", directed=False,n_iter=100)
  graph_draw(g, output="Rand.pdf")
  # Output adjacency matrix
  A = adjacency(g).astype(int).toarray()
  #print A.shape
  #print A
  np.savetxt('Rand.txt',A,fmt='%i',header=str(A.shape[0]),comments='')
