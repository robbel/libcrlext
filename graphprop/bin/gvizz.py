#!/usr/bin/python

#
#
#
#

import numpy as np
import pylab as py
from graph_tool.all import *

py.ion()

def plot_data(data):
  py.clf()
  py.plot(data)
  py.draw()
  #py.savefig("data-%.8d.png"%counter)

# def plot_data(data):
#   py.clf()
#   py.plot(data)
#   py.draw()
#   #py.savefig("data-%.8d.png"%counter)
 
if __name__ == "__main__":
  # Read file with numpy 
  Adj = np.loadtxt("Gnp100.txt",skiprows=1,dtype=np.int);
  # Generate graph from adjacency matrix
  g = Graph()
  g.add_edge_list(np.transpose(Adj.nonzero()))

  # perhaps add properties to graph (agent/target) -- can be stored to file, btw
  # note: can filter instead of remove edges/vertices

  counter = 0
  while True:
    try:
      tmp = raw_input().strip().split()
      data = np.array(tmp, dtype=np.double)
    except EOFError:
      print "Input has terminated! Exiting"
      exit()
    except ValueError:
      print "Invalid input, skipping.  Input was: %s"%tmp
      continue
 
    print "Plotting plot number %d"%counter
    plot_data(data)
    counter += 1
