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

#include <cpputil.hpp>

#include "crl/outcomes.hpp"

using namespace crl;
using namespace std;
using namespace cpputil;

_StepOutcome::_StepOutcome(const vector<int>& deltas)
: _deltas(deltas) {

}

bool _StepOutcome::match(const State& s, const State& sp) {
	if (!sp)
		return false;
	for (Size i=0; i<s.size(); i++) {
		if (sp.getFactor(i)-s.getFactor(i) != _deltas[i])
			return false;
	}
	return true;
}

_FixedOutcome::_FixedOutcome(const State& s)
: _s(s) {

}

bool _FixedOutcome::match(const State& s, const State& sp) {
	return sp && sp == _s;
}

bool _TerminalOutcome::match(const State& s, const State& sp) {
	return !sp;
}

_OutcomeTable::_OutcomeTable(const Domain& domain, const std::vector<Outcome>& outcomes)
: _domain(domain), _outcomes(outcomes),
  _outcomeCounts(domain, std::vector<Size>(outcomes.size(), 0)),
  _outcomeDistributions(domain, std::vector<Probability>(outcomes.size(), 0)) {

}

bool _OutcomeTable::observe(const State& s, const Action& a, const Observation& o) {
	State sp = o->getState();
//	if (!sp)
//		return false;
	Size total = 0;
	vector<Size>& counts = _outcomeCounts.getValue(s, a);

	for (Size i=0; i<_outcomes.size(); i++) {
		if (_outcomes[i]->match(s, sp)) {
			counts[i] += 1;
		}
		total += counts[i];
	}

	return true;
}

void _OutcomeTable::print() {
	StateIterator sitr(new _StateIncrementIterator(_domain));
	while (sitr->hasNext()) {
		State s = sitr->next();
		cerr << "for state " << s << endl;
		ActionIterator aitr(new _ActionIncrementIterator(_domain));
		while (aitr->hasNext()) {
			Action a = aitr->next();
			cerr << " for action " << a << endl;
			vector<Size>& counts = _outcomeCounts.getValue(s, a);
			for (Size i=0; i<_outcomes.size(); i++) {
				cerr << "  outcome " << i << " = " << counts[i] << endl;
			}
		}
	}
}

_Cluster::_Cluster(const Domain& domain, OutcomeTable outcome_table)
: _domain(domain), _outcome_table(outcome_table),
  _outcome_counts(domain, vector<Size>(_outcome_table->numOutcomes(), 1)),
  _outcome_totals(domain, 0),
  _outcome_probs(domain, vector<Probability>(_outcome_table->numOutcomes())), _num_states(0) {
	ActionIterator aitr = ActionIterator(new _ActionIncrementIterator(_domain));
	while (aitr->hasNext()) {
		Action a = aitr->next();
		Size total = 0;
		vector<Size>& counts = _outcome_counts.getValue(a);
		for (Size i=0; i<counts.size(); i++)
			total += counts[i];
		_outcome_totals.setValue(a, total);
		Probability totalInv = 1.0/total;
		vector<Probability>& probs = _outcome_probs.getValue(a);
		for (Size i=0; i<counts.size(); i++)
			probs[i] = totalInv*counts[i];
	}
}

_Cluster::_Cluster(const Domain& domain, OutcomeTable outcome_table, _FActionTable<std::vector<Size> > _outcome_priors)
: _domain(domain), _outcome_table(outcome_table), _outcome_counts(_outcome_priors),
  _outcome_totals(domain, 0),
  _outcome_probs(domain, vector<Probability>(_outcome_table->numOutcomes())), _num_states(0) {
	ActionIterator aitr = ActionIterator(new _ActionIncrementIterator(_domain));
	while (aitr->hasNext()) {
		Action a = aitr->next();
		Size total = 0;
		vector<Size>& counts = _outcome_counts.getValue(a);
		for (Size i=0; i<counts.size(); i++)
			total += counts[i];
		_outcome_totals.setValue(a, total);
		Probability totalInv = 1.0/total;
		vector<Probability>& probs = _outcome_probs.getValue(a);
		for (Size i=0; i<probs.size(); i++)
			probs[i] = totalInv*counts[i];
	}
}

void _Cluster::addState(const State& s) {
	_num_states++;
	ActionIterator aitr(new _ActionIncrementIterator(_domain));
	while (aitr->hasNext()) {
		Action a = aitr->next();
		vector<Size>& cluster_action_counts = _outcome_counts.getValue(a);
		vector<Size>& total_action_counts = _outcome_table->getOutcomeCounts(s, a);
		Size total = _outcome_totals.getValue(a);
		for (Size i=0; i<_outcome_table->numOutcomes(); i++) {
			cluster_action_counts[i] += total_action_counts[i];
			total += total_action_counts[i];
		}
		_outcome_totals.setValue(a, total);
		Probability totalInv = 1.0/total;
		vector<Probability>& probs = _outcome_probs.getValue(a);
		for (Size i=0; i<probs.size(); i++)
			probs[i] = totalInv*cluster_action_counts[i];
	}
}

void _Cluster::removeState(const State& s) {
	_num_states--;
	ActionIterator aitr(new _ActionIncrementIterator(_domain));
	while (aitr->hasNext()) {
		Action a = aitr->next();
		vector<Size>& cluster_action_counts = _outcome_counts.getValue(a);
		vector<Size>& total_action_counts = _outcome_table->getOutcomeCounts(s, a);
		Size total = _outcome_totals.getValue(a);
		for (Size i=0; i<_outcome_table->numOutcomes(); i++) {
			cluster_action_counts[i] -= total_action_counts[i];
			total -= total_action_counts[i];
		}
		_outcome_totals.setValue(a, total);
		Probability totalInv = 1.0/total;
		vector<Probability>& probs = _outcome_probs.getValue(a);
		for (Size i=0; i<probs.size(); i++)
			probs[i] = totalInv*cluster_action_counts[i];
	}
}

Size _Cluster::size() {
	return _num_states;
}

Probability _Cluster::P(const Action& a, const Outcome& o) {
	Size index = _outcome_table->getOutcomeIndex(o);
	return _outcome_probs.getValue(a)[index];
}

Probability _Cluster::logP(const State& s) {
	Probability log_p = 0;
	ActionIterator aitr(new _ActionIncrementIterator(_domain));
	while (aitr->hasNext()) {
		Action a = aitr->next();
		vector<Size>& outcome_counts = _outcome_table->getOutcomeCounts(s, a);
		for (Size outcome_index=0; outcome_index<_outcome_table->numOutcomes(); outcome_index++) {
			Outcome o = _outcome_table->getOutcome(outcome_index);
			Probability outcome_likelihood = P(a, o);
			Probability log_likelihood = log(outcome_likelihood);
			log_p += log_likelihood * outcome_counts[outcome_index];
			//cerr << log_p << endl;
		}
	}
	return log_p;
}

void _Cluster::print() {
	ActionIterator aitr(new _ActionIncrementIterator(_domain));
	while (aitr->hasNext()) {
		Action a = aitr->next();
		cerr << " for action " << a << endl;
		vector<Size>& counts = _outcome_counts.getValue(a);
		for (Size i=0; i<_outcome_table->numOutcomes(); i++) {
			cerr << "  outcome " << i << " = " << counts[i] << endl;
		}
	}
}



