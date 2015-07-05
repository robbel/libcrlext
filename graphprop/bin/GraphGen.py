#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright 2014 Christopher Ho
# 
# License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.
# 
# The software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

from igraph import *
import sys

file_name = sys.argv[1]
type = sys.argv[2]
n = int(sys.argv[3])

A = Graph.Full(n)

if type == "barabasi":
	m = int(sys.argv[4])
	A = Graph.Barabasi(n,m,directed=True);
elif type == "k-regular":
	k = int(sys.argv[4])
	A = Graph.K_Regular(n,k,directed=True);
elif type == "sbm":
	print "Please define pref_matrix and block_sizes in this script by hand"
	#say: pref_mat = [[1,2],[3,4]]
	block_sizes = 0;
	pref_matrix = [[2*math.log(block_sizes[i])/block_sizes[i] if i == j else \
		math.log(len(block_sizes))/len(block_sizes) for i in range(len(block_sizes))] \
		for j in range(len(block_sizes))]

	A = Graph.SBM(n,pref_matrix,block_sizes,directed=True);
elif type == "er" or type == "erdos-renyi" or type == "gnp":
	if len(sys.argv) > 4:
		p = float(sys.argv[4])
		if p == 0:
			p = 1.01*math.log(n)/(n-1);
	else:
		p = 1.01*math.log(n)/(n-1);
	A = Graph.Erdos_Renyi(n,p);
elif type == "power law" or type == "scale free":
	m = int(sys.argv[4])
	exp_in = float(sys.argv[5])
	exp_out = float(sys.argv[6])
	A = Graph.Static_Power_Law(n,m,exp_in,exp_out);
else:
	print "Usage: python GraphGen.py [file name] [graph type] [number of nodes] [args]"
	print "Graph type = {\"barabasi\", \"k-regular\", \"sbm\", \"gnp\", \"power law\"}"

B = Graph.get_adjacency(A);
C_ = str(B);

C = C_.strip(']').strip('[').replace("]\n [","\n").replace(", "," ")

f = open(file_name,'w')

f.write(str(n)+'\n');
f.write(C)

f.close()
