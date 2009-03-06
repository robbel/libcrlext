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
#include <crl/vi.hpp>
#include "crl/cluster_gibbs.hpp"
#include "crl/outcomes.hpp"

using namespace std;
using namespace crl;

float getX(State s, int offset) {
	return s.getFactor(0)*12;
}
float getY(State s, int offset) {
	if (s.size() > 1)
		return s.getFactor(1)*45+offset*200;
	return offset*45;
}

void _OutcomeClusterLearner::makeClusterBOSSVis(diastream& os, int offset, ClusterMDP cmdp) {
	vector<Cluster> seen_clusters;
	
	FQTable qtable(new _FQTable(_domain));
	VIPlanner vip(new _VIPlanner(cmdp, .01, 1, qtable));
	vip->plan();
	
	
	
	os << DiaBeginLayer("States");
	StateIterator sitr = cmdp->S();
	while (sitr->hasNext()) {
		State s = sitr->next();
		float sx = getX(s, offset);
		float sy = getY(s, offset);
		
		if (s.getIndex() == 0) {
			ostringstream tos;
			tos << "LL = " << cmdp->logP();
			os << DiaText(sx, sy-1, tos.str());
		}
		
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
		
		os << DiaBox(sx, sy, sx+10, sy+35, color);
	}
	
	
	sitr->reset();
	while (sitr->hasNext()) {
		State s = sitr->next();
		if (s.size() == 3 && s.getFactor(2) == 1)
			continue;
		os << DiaBeginGroup();
		Cluster c = cmdp->getCluster(s);
		float sx = getX(s, offset);
		float sy = getY(s, offset);
		
		Action best_action = vip->getAction(s);
		
		ostringstream tos;
		tos << s << "(" << best_action << ") = " << qtable->getV(s);
		os << DiaText(sx, sy, tos.str());
		
		os << DiaText(sx+1, sy+1, "a,o(s)");
		os << DiaText(sx+5, sy+1, "a,o(c)");
		
//		float a_offset = 0;
		ActionIterator aitr = cmdp->A(s);
		while (aitr->hasNext()) {
			Action a = aitr->next();
			
			
			vip->backupStateAction(s, a);
			
			int a_index = a.getIndex();
			
			vector<Size>& counts = _outcome_table->getOutcomeCounts(s, a);
			for (Size i=0; i<counts.size(); i++) {
				ostringstream tos;
				tos << a_index << "," << i << ":" << counts[i];
				os << DiaText(sx+1, sy+a_index*8+i+2, tos.str());
			}
			
			vector<Size>& counts2 = c->getCounts(a);
			for (Size i=0; i<counts2.size(); i++) {
				Outcome o = cmdp->getOutcome(i);
				State n = o->apply(s);
				Probability p = cmdp->T(s, a)->P(n);
				ostringstream tos;
				tos << a_index << "," << i << ":" << counts2[i] << "/" << setprecision(2) << p;
				os << DiaText(sx+5, sy+a_index*8+i+2, tos.str());
			}
			
			
				
			Reward r = cmdp->R(s, a);
			ostringstream tos;
			tos << r << ";" << qtable->getQ(s, a);
			os << DiaText(sx+2, sy+a_index*8+counts2.size()+2, tos.str());
			/*
			StateDistribution sd = cmdp->T(s, a);
			StateIterator nitr = sd->iterator();
			while (nitr->hasNext()) {
				State n = nitr->next();
				float y_offset = a_offset;
				float x_offset = 0;
				if (a_index == 0) {
					x_offset += 10;
					y_offset += 5;
				}
				Probability p = sd->P(n);
				
				float ex = getX(n, offset);
				float ey = getY(n, offset);
				float arrow_y = sy+y_offset;
				float arrow_x = sx+x_offset;
				if (a_index == 0)
					os << DiaArc(arrow_x, arrow_y, ex+x_offset, ey+y_offset+1, 5, "#FF0000");
				if (a_index == 1)
					os << DiaArc(arrow_x, arrow_y, ex+x_offset, ey+y_offset+1, 5, "#0000FF");
				
				ostringstream tos;
				tos << setprecision(2) << fixed << p << "(" << r << ")";
				if (a_index == 1)
					os << DiaText(arrow_x-2, arrow_y-.5, tos.str());
				else
					os << DiaText(arrow_x, arrow_y-.5, tos.str());
				a_offset += 2;
			}
				*/
		}
		
		os << DiaEndGroup();
	}
	os << DiaEndLayer();
	
}
