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

#include <cpputil.hpp>
#include "crl/boss.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;

_BOSSPlanner::_BOSSPlanner(Reward epsilon, float gamma)
: _VIPlanner(MDP(), epsilon, gamma) {

}

_BOSSPlanner::_BOSSPlanner(Reward epsilon, float gamma, QTable qtable)
: _VIPlanner(MDP(), epsilon, gamma, qtable) {

}

Reward _BOSSPlanner::evaluateStateAction(const State& s, const Action& a) {
	Reward q = numeric_limits<Reward>::max()*-1;
	ContainerIterator<MDP,vector<MDP> > mitr(_mdps);
	while (mitr.hasNext()) {
		_mdp = mitr.next();
		Reward pq = _VIPlanner::evaluateStateAction(s, a);
		if (pq > q)
			q = pq;
	}
	return q;
}

void _BOSSPlanner::setMDPs(vector<MDP> mdps) {
	_mdps = mdps;
}

void _BOSSPlanner::plan() {
	_mdp = *(_mdps.begin());
	_VIPlanner::plan();
}

