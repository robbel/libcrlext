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

#include <math.h>
#include <iostream>
#include "crl/crl.hpp"
#include "crl/vi.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;

_VIPlanner::_VIPlanner(MDP mdp, Reward epsilon, float gamma)
: _mdp(mdp), _epsilon(epsilon), _gamma(gamma) {

}
_VIPlanner::_VIPlanner(MDP mdp, Reward epsilon, float gamma, QTable qtable)
: _mdp(mdp), _qtable(qtable), _epsilon(epsilon), _gamma(gamma) {

}
Reward _VIPlanner::backupState(const State& s, ActionIterator& aitr) {
//	cout << "backing up " << s << endl;
	aitr->reset();
	Reward error_inf = 0;
	while (aitr->hasNext()) {
		Action a = aitr->next();
		Reward error = backupStateAction(s, a);
		if (error > error_inf)
			error_inf = error;
	}
	return error_inf;
}
Reward _VIPlanner::backupStateAction(const State& s, const Action& a) {

	Reward q = evaluateStateAction(s, a);

	Reward error = fabs(q-_qtable->getQ(s, a));
	_qtable->setQ(s, a, q);

	return error;
}

Reward _VIPlanner::evaluateStateAction(const State& s, const Action& a) {
	StateDistribution sd = _mdp->T(s, a);
	StateIterator nitr = sd->iterator();
	Reward q = 0;
	while (nitr->hasNext()) {
		State n = nitr->next();
		Probability p = sd->P(n);
		Reward v = _qtable->getV(n);
		q += p*v;
	}

	q *= _gamma;
	q += _mdp->R(s, a);
	return q;
}

int _VIPlanner::plan(StateIterator sitr, ActionIterator aitr) {
	//cerr << "+_VIPlanner::plan" << endl;
	int num_iterations = 0;
	Reward error_inf = 0;

	do {
		error_inf = 0;
		sitr->reset();

		while (sitr->hasNext()) {
			State s = sitr->next();
			Reward error = backupState(s, aitr);
			if (error > error_inf)
				error_inf = error;
		}
		num_iterations++;
//		cout << num_iterations << " : error_inf = " << error_inf << endl;
//		cout << " " << error_inf << endl;
	}
	while (error_inf > _epsilon);
	//cerr << "-_VIPlanner::plan" << endl;
	return num_iterations;
}
int _VIPlanner::plan() {
	StateIterator sitr = _mdp->S();
	ActionIterator aitr = _mdp->A();
	return plan(sitr, aitr);
}
Action _VIPlanner::getAction(const State& s) {
	Action a = _qtable->getBestAction(s);
	return a;
}
QTable _VIPlanner::getQTable() {
	return _qtable;
}

_VIPlannerAgent::_VIPlannerAgent(VIPlanner planner, Learner learner)
: _Agent(planner, learner) {

}

bool _VIPlannerAgent::observe(const Observation& o) {
	bool learned;
	if ((learned = _Agent::observe(o))) {
		getVIPlanner(_planner)->plan();
	}
	return learned;
}

