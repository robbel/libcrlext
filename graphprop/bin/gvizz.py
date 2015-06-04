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

"""Animated visualization for the graphprop problem.

Reads state information from STDIN and animates the graph accordingly.

Run as ./graphprop <config> <graph_file> --enable-stdout | ./gvizz <graph_file> [<offscreen>]
"""

__author__ = 'Philipp Robbel'

import numpy as np
from graph_tool.all import *
import sys, os, os.path
from gi.repository import Gtk, Gdk, GdkPixbuf, GObject

# The one and only graph instance
g = Graph()
# Vertex color scheme
S = [1, 1, 1, 1]           # White color
I = [0, 0, 0, 1]           # Black color
R = [0.5, 0.5, 0.5, 1.]    # Grey color (will not actually be drawn)
# For frame output
count = 0
max_count = 0
# The main output window
win = 0

# Main gtk loop
def update_state():
  """Highlight the current state and action in the graph"""
  global g, S, I, R
  global win
  data = np.empty(shape=(0,0), dtype=np.int)

  try:
    tmp = raw_input().strip()
    if tmp.startswith("S: "):
      data = np.array(tmp[4:-1].split(), dtype=np.int)
  except EOFError:
    print "STDIN input has terminated. Close window to exit."
    return False
  except ValueError:
    print "Invalid input, skipping. Input was: %s" % tmp
    #return True
 
  #newly_infected.a = False
  if data.size < g.num_vertices():
    print "Warning: input array size does not match number of graph vertices."
    data = np.append(data,np.zeros(g.num_vertices()-data.size))
  newly_infected.a = data.astype(bool)

  # The following will force the re-drawing of the graph, and issue a
  # re-drawing of the GTK window.
  win.graph.regenerate_surface(lazy=False)
  win.graph.queue_draw()

  # If doing an offscreen animation, dump frame to disk
  if offscreen:
    global count, max_count
    pixbuf = win.get_pixbuf()
    pixbuf.savev(r'./frames/sirs%06d.png' % count, 'png', [], [])
    if count > max_count:
      sys.exit(0)
    count += 1

  # Return True so that the main loop will call this function more than once.
  return True

  # TODO: Add keyboard handler to save figure
  #py.savefig("data-%.8d.png"%counter)

   
if __name__ == "__main__":
  if len(sys.argv) < 2:
    print "Usage: python gvizz.py <filename> [<offscreen>]"
    sys.exit(0)

  # Read file with numpy 
  Adj = np.loadtxt(sys.argv[1],skiprows=1,dtype=np.int);
  # Detect directed or undirected graph
  if (Adj == np.transpose(Adj)).all():
    g.set_directed(False)
    Adj = np.triu(Adj)
  # Generate graph from adjacency matrix
  g.add_edge_list(np.transpose(Adj.nonzero()))
  # Initialize all vertices to the S state
  state = g.new_vertex_property("vector<double>")
  for v in g.vertices():
    state[v] = S

  # Automatic graph layout
  pos = sfdp_layout(g)

  # Newly infected nodes will be highlighted in red
  newly_infected = g.new_vertex_property("bool")

  # If True, the frames will be dumped to disk as images.
  offscreen = sys.argv[2] == "offscreen" if len(sys.argv) > 2 else False
  max_count = 500
  if offscreen and not os.path.exists("./frames"):
    os.mkdir("./frames")
    
  # This creates a GTK+ window with the initial graph layout
  if not offscreen:
    win = GraphWindow(g, pos, geometry=(500, 400),
                      edge_color=[0.6, 0.6, 0.6, 1],
                      vertex_fill_color=state,
                      vertex_halo=newly_infected,
                      vertex_halo_color=[0.8, 0, 0, 0.6])
  else:
    count = 0
    win = Gtk.OffscreenWindow()
    win.set_default_size(500, 400)
    win.graph = GraphWidget(g, pos,
                            edge_color=[0.6, 0.6, 0.6, 1],
                            vertex_fill_color=state,
                            vertex_halo=newly_infected,
                            vertex_halo_color=[0.8, 0, 0, 0.6])
    win.add(win.graph)


  # TODO: add properties to graph (agent/target/vertex layout) -- can be stored to file
  # Note: can filter instead of removing edges/vertices during animation

  # Bind the 'idle' callback.
  cid = GObject.idle_add(update_state)

  # We will give the user the ability to stop the program by closing the window.
  win.connect("delete_event", Gtk.main_quit)

  # Actually show the window, and start the main loop.
  win.show_all()
  Gtk.main()
