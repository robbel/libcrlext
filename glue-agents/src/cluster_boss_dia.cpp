/*
    Copyright 2009 Rutgers University
    Copyright 2009 John Asmuth

    This file is part of CRL:RL-Glue:bayes.

    CRL:RL-Glue:bayes is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    CRL:RL-Glue:bayes is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with CRL:RL-Glue:bayes.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <iomanip>
#include <sstream>
#include <diastream.hpp>

#include <crl/crl.hpp>
#include "crl/cluster_gibbs.hpp"
#include "crl/outcomes.hpp"

using namespace std;
using namespace crl;

float getX(State s, int offset) {
	return s.getFactor(0)*20;
}
float getY(State s, int offset) {
	if (s.size() > 1)
		return s.getFactor(1)*20+offset*15;
	return offset*15;
}

void _OutcomeClusterLearner::makeClusterBOSSVis(diastream& os, int offset, const char* path, ClusterMDP cmdp) {

	
	
	vector<Cluster> seen_clusters;
	
	os << DiaBeginLayer("States");
	StateIterator sitr = cmdp->S();
	while (sitr->hasNext()) {
		State s = sitr->next();
		float sx = getX(s, offset);
		float sy = getY(s, offset);
		
		Cluster c = cmdp->getCluster(s);
		Size cluster_index;
		for (cluster_index=0;
			 cluster_index<seen_clusters.size() && seen_clusters[cluster_index] != c;
			 cluster_index++);
		if (cluster_index >= seen_clusters.size())
			seen_clusters.push_back(c);
		string color = "#FFAAAA";
		if (cluster_index == 1)
			color = "#AAFFAA";
		if (cluster_index == 2)
			color = "#AAAAFF";
		if (cluster_index == 3)
			color = "#FFAAFF";
		if (cluster_index == 4)
			color = "#FFFFAA";
		
		os << DiaBox(sx, sy, sx+10, sy+10, color);
	}
	
	
	sitr->reset();
	while (sitr->hasNext()) {
		os << DiaBeginGroup();
		State s = sitr->next();
		Cluster c = cmdp->getCluster(s);
		float sx = getX(s, offset);
		float sy = getY(s, offset);
			
		ostringstream tos;
		tos << s;
		os << DiaText(sx, sy, tos.str());
		
		os << DiaText(sx+2, sy+1, "a,o(s)");
		os << DiaText(sx+6, sy+1, "a,o(c)");
		
		float a_offset = 0;
		int a_index = 0;
		ActionIterator aitr = cmdp->A(s);
		while (aitr->hasNext()) {
			Action a = aitr->next();
			
			vector<Size>& counts = _outcome_table->getOutcomeCounts(s, a);
			for (Size i=0; i<counts.size(); i++) {
				ostringstream tos;
				tos << a.getIndex() << "," << i << ":" << counts[i];
				os << DiaText(sx+2, sy+a_offset+i+2, tos.str());
			}
			
			vector<Size>& counts2 = c->getCounts(a);
			for (Size i=0; i<counts2.size(); i++) {
				ostringstream tos;
				tos << a.getIndex() << "," << i << ":" << counts2[i];
				os << DiaText(sx+6, sy+a_offset+i+2, tos.str());
			}
			
			
			StateDistribution sd = cmdp->T(s, a);
			StateIterator nitr = sd->iterator();
			while (nitr->hasNext()) {
				State n = nitr->next();
				Probability p = sd->P(n);
				
				float ex = getX(n, offset);
				float ey = getY(n, offset);
				float y_offset = a_offset;
				float x_offset = 0;
				if (a_index == 0) {
					x_offset += 10;
					y_offset += 5;
				}
				float arrow_y = sy+y_offset;
				float arrow_x = sx+x_offset;
				if (a_index == 0)
					os << DiaArc(arrow_x, arrow_y, ex+x_offset, ey+y_offset+1, 5, "#FF0000");
				if (a_index == 1)
					os << DiaArc(arrow_x, arrow_y, ex+x_offset, ey+y_offset+1, 5, "#0000FF");
				
				ostringstream tos;
				tos << setprecision(2) << fixed << p;
				os << DiaText(arrow_x, arrow_y-.5, tos.str());
				
				a_offset += 2;
			}
			a_index++;
		}
		
		os << DiaEndGroup();
	}
	os << DiaEndLayer();
	
}
