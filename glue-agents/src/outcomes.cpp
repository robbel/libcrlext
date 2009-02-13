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

#include <math.h>
#include "crl/outcomes.hpp"

using namespace crl;
using namespace std;

_StepOutcome::_StepOutcome(const vector<int>& deltas)
: _deltas(deltas) {

}

bool _StepOutcome::match(const State& s, const State& sp) {
	for (Size i=0; i<s.size(); i++) {
		if (sp.getFactor(i)-s.getFactor(i) != _deltas[i])
			return true;
	}
	return false;
}

_FixedOutcome::_FixedOutcome(const State& s)
: _s(s) {

}

bool _FixedOutcome::match(const State& s, const State& sp) {
	return sp == _s;
}
_OutcomeTable::_OutcomeTable(const Domain& domain, const std::vector<Outcome>& outcomes)
: _domain(domain), _outcomes(outcomes),
  _outcomeCounts(domain, std::vector<Size>(outcomes.size(), 0)),
  _outcomeDistributions(domain, std::vector<Probability>(outcomes.size(), 0)) {

}

bool _OutcomeTable::observe(const State& s, const Action& a, const Observation& o) {
	State sp = o->getState();
	Size total = 0;
	vector<Size>& counts = _outcomeCounts.getValue(s, a);
	for (Size i=0; i<_outcomes.size(); i++) {
		if (_outcomes[i]->match(s, sp))
			counts[i] += 1;
		total += counts[i];
	}
	double normInv = 1.0/total;
	vector<Probability>& probabilities = _outcomeDistributions.getValue(s, a);
	for (Size i=0; i<_outcomes.size(); i++)
		probabilities[i] = counts[i]*normInv;

	return true;
}

Probability _OutcomeTable::outcomesGivenDistribution(const State& s, vector<_FActionTable<Probability> > dist) {
	Probability p = 1;
	ActionIterator itr(new _ActionIncrementIterator(_domain));
	for (Size i=0; i<dist.size(); i++) {
		_FActionTable<Probability>& ptable = dist[i];
		while (itr->hasNext()) {
			Action a = itr->next();
			vector<Size>& counts = _outcomeCounts.getValue(s, a);
			p *= pow(ptable.getValue(a), counts[i]);
		}
	}

	return p;
}

vector<_FActionTable<Probability> > _OutcomeTable::clusterDistribution(StateIterator sitr) {
	std::vector<_FActionTable<Size> > counts(_outcomes.size(), _FActionTable<Size>(_domain, 0));
	//_FActionTable
	sitr->reset();
	ActionIterator aitr(new _ActionIncrementIterator(_domain));
	while (sitr->hasNext()) {
		State s = sitr->next();
		aitr->reset();
		while (aitr->hasNext()) {
			Action a = aitr->next();
			for (Size i=0; i<_outcomes.size(); i++) {
				Size count = counts[i].getValue(a);
				counts[i].setValue(a, count+_outcomeCounts.getValue(s, a)[i]);
			}
		}
	}
	vector<_FActionTable<Probability> > dist(_outcomes.size(), _FActionTable<Probability>(_domain));

	aitr->reset();
	while (aitr->hasNext()) {
		Action a = aitr->next();
		for (Size i=0; i<_outcomes.size(); i++) {

		}
	}
	return dist;
}
