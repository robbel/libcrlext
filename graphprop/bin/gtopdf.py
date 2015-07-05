#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright 2015 Philipp Robbel
# Idea from http://micha.gd/code/2011/09/27/dead-simple-real-time-plotting-with-c-c-and-python/
# Based in part on graph animation example from graph-tool documentation.
# 
# License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.
# 
# The software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

"""Convert graph adjacency matrix from given file into PDF.

Run as ./gvizz <graph_file> <pdf_file>
"""
import numpy as np
from graph_tool.all import *
import sys, os, os.path

# The one and only graph instance
g = Graph()

if __name__ == "__main__":
  if len(sys.argv) < 3:
    print "Usage: python gtopdf.py <graph_file> <pdf_file>"
    sys.exit(0)

  # Read graph file with numpy 
  Adj = np.loadtxt(sys.argv[1],skiprows=1,dtype=np.int);
  # Detect directed or undirected graph
  if (Adj == np.transpose(Adj)).all():
    g.set_directed(False)
    Adj = np.triu(Adj)
  # Generate graph from adjacency matrix
  g.add_edge_list(np.transpose(Adj.nonzero()))
  # Automatic graph layout
  pos = sfdp_layout(g)

  # Draw graph to PDF
  graph_draw(g, pos=pos, output=sys.argv[2])
