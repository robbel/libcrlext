/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth

    This file is part of CRL.

    CRL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    CRL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with CRL.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include "crl/prioritized_sweeping.hpp"

using namespace std;
using namespace crl;


_PSPlanner::_PSPlanner(const MDP& mdp, Reward epsilon, float gamma)
: _VIPlanner(mdp, epsilon, gamma) {
	
}

_PSPlanner::_PSPlanner(const MDP& mdp, Reward epsilon, float gamma, QTable qtable)
: _VIPlanner(mdp, epsilon, gamma, qtable) {
	
}

int _PSPlanner::sweep(priority_queue& pq, ActionIterator& aitr) {
	if (pq.size() == 0)
		return 0;
	iterator itr = pq.begin();
	int count = 0;
	while (itr->first > _epsilon) {
		State s = itr->second;
		pq.erase(itr);
		StateIterator pitr = _mdp->predecessors(s);
		
		while (pitr->hasNext()) {
			State p = pitr->next();
			Reward error = backupState(p, aitr);
			priority_queue::value_type vt(error, p);
			pq.insert(vt);
			count++;
		}
	}
	return count;
}
