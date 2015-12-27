#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright 2015 Philipp Robbel
# 
# License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.
# 
# The software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

"""Convert graph adjacency matrix from given file into PDF.

Run as ./gtopdf2 <graph_file> <pdf_file>
"""
import numpy as np
from graph_tool.all import *
from xml.etree.ElementTree import ElementTree
import sys, os, os.path

# The one and only graph instance
g = Graph()

if __name__ == "__main__":
  if len(sys.argv) < 4:
    print "Usage: python gtopdf2.py <config> <graph_file> <pdf_file>"
    sys.exit(0)

  #Agents = [9,45,42,49,6,13,19,81,78,83,95,73,75,80,52,58,31,56,16,71,46,27,68,47,61,69,39,10,67,20,1,76,87,11,60,96,36,37,55,65,38,54,66,14,4,97,90,3,86,0]
  #Targets = [5,8,93,7,63,2,99,59,22,92,34,53,74,12,32,15,57,40,88,64,33,91,28,84,24,72,85,23,25,79,44,48,26,35,30,51,89,94,17,77,50,41,70,82,18,21,62,29,98,43]

  # Read graph file with numpy 
  Adj = np.loadtxt(sys.argv[2],skiprows=1,dtype=np.int);
  # Detect directed or undirected graph
  if (Adj == np.transpose(Adj)).all():
    g.set_directed(False)
    Adj = np.triu(Adj)
  # Generate graph from adjacency matrix
  g.add_edge_list(np.transpose(Adj.nonzero()))
  # Automatic graph layout
  pos = arf_layout(g)

  # Add some vertex properties for drawing
  view = g.new_vertex_property("int")
  # Read config xml file for agent locations
  xml_parser = ElementTree()
  tree = xml_parser.parse(sys.argv[1])
  locs = tree.find('Agents').text
  # Size nodes by their "importance"
  deg = g.degree_property_map("out")
  deg.a = 12*np.sqrt(deg.a/min(deg.a)) # normalize 
  #deg.a = 5 * (np.sqrt(deg.a) * 2 + 0.5)
  for v in [g.vertex(x) for x in locs.split(',')]:
    deg[v] = deg[v]*1.8
    view[v] = 1

  # Draw graph to PDF
  graph_draw(g, vertex_text=g.vertex_index, vertex_size=deg, vertex_shape=view, pos=pos, output_size=(1024, 768), output=sys.argv[3])
