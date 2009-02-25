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
#include <iomanip>

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

#include <cpputil.hpp>

#include "crl/outcomes.hpp"

using namespace crl;
using namespace std;
using namespace cpputil;

_StepOutcome::_StepOutcome(Domain domain, const vector<int>& deltas)
: _domain(domain), _deltas(deltas) {

}

bool _StepOutcome::match(const State& s, const State& sp) {
	return (sp && apply(s) == sp);
	/*
	if (!sp)
		return false;
	for (Size i=0; i<s.size(); i++) {
		Factor
		if (sp.getFactor(i)+_deltas[i] > 
		if (sp.getFactor(i)-s.getFactor(i) != _deltas[i])
			return false;
	}
	return true;
	*/
}

State _StepOutcome::apply(const State& s) {
	State sp(s);
	for (Size i=0; i<s.size(); i++) {
		Factor f = s.getFactor(i)+_deltas[i];
		const RangeVec& ranges = _domain->getStateRanges();
		if (ranges[i].check(f))
			sp.setFactor(i, f); 
	}
	return sp;
}

_FixedOutcome::_FixedOutcome(const State& s)
: _s(s) {

}

bool _FixedOutcome::match(const State& s, const State& sp) {
	return (sp && apply(s) == sp);
}

State _FixedOutcome::apply(const State& s) {
	return _s;
}

bool _TerminalOutcome::match(const State& s, const State& sp) {
	return !sp;
}

State _TerminalOutcome::apply(const State& s) {
	return State();
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
	//Size total = 0;
	vector<Size>& counts = _outcomeCounts.getValue(s, a);

	for (Size i=0; i<_outcomes.size(); i++) {
		if (_outcomes[i]->match(s, sp)) {
			counts[i] += 1;
			break;
		}
		//total += counts[i];
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

_Cluster::_Cluster(const Domain& domain, OutcomeTable outcome_table, gsl_rng* gsl_random)
: _domain(domain), _outcome_table(outcome_table),
  _outcome_counts(domain, vector<Size>(_outcome_table->numOutcomes(), 1)),
  _outcome_totals(domain, 0),
  _outcome_probs(domain, vector<Probability>(_outcome_table->numOutcomes())),
  _outcome_probs_no_model(domain, vector<Probability>(_outcome_table->numOutcomes())),
  _num_states(0), _gsl_random(gsl_random) {
	ActionIterator aitr = ActionIterator(new _ActionIncrementIterator(_domain));
	while (aitr->hasNext()) {
		Action a = aitr->next();
		Size total = 0;
		vector<Size>& counts = _outcome_counts.getValue(a);
		for (Size i=0; i<counts.size(); i++)
			total += counts[i];
		_outcome_totals.setValue(a, total);
		calcProbs(a);
	}
}

_Cluster::_Cluster(const Domain& domain, OutcomeTable outcome_table, _FActionTable<std::vector<Size> > _outcome_priors, gsl_rng* gsl_random)
: _domain(domain), _outcome_table(outcome_table), _outcome_counts(_outcome_priors),
  _outcome_totals(domain, 0),
  _outcome_probs(domain, vector<Probability>(_outcome_table->numOutcomes())),
  _outcome_probs_no_model(domain, vector<Probability>(_outcome_table->numOutcomes())),
  _num_states(0), _gsl_random(gsl_random) {
	ActionIterator aitr = ActionIterator(new _ActionIncrementIterator(_domain));
	while (aitr->hasNext()) {
		Action a = aitr->next();
		Size total = 0;
		vector<Size>& counts = _outcome_counts.getValue(a);
		for (Size i=0; i<counts.size(); i++)
			total += counts[i];
		_outcome_totals.setValue(a, total);
		calcProbs(a);
	}
}

void _Cluster::setGSLRandom(gsl_rng* gsl_random) {
	_gsl_random = gsl_random;
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
		calcProbs(a);
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
		calcProbs(a);
	}
}

void _Cluster::calcProbs(const Action& a) {
	vector<Size>& cluster_action_counts = _outcome_counts.getValue(a);
	Size total = _outcome_totals.getValue(a);
	Probability totalInv = 1.0/total;
	vector<Probability>& probs_no_model = _outcome_probs_no_model.getValue(a);
	for (Size i=0; i<probs_no_model.size(); i++)
		probs_no_model[i] = totalInv*cluster_action_counts[i];
		
		
	Size K = cluster_action_counts.size();
	double* dirichlet_alpha = new double[K];
	double* dirichlet_theta = new double[K];
	bool b = false;
	for (Size i=0; i<K; i++) {
		dirichlet_alpha[i] = cluster_action_counts[i];
		if (b)
			cerr << " " << dirichlet_alpha[i];
	}
	if (b)
		cerr << endl;
	gsl_ran_dirichlet(_gsl_random, K, dirichlet_alpha, dirichlet_theta);
	vector<Probability>& probs = _outcome_probs.getValue(a);
	for (Size i=0; i<probs.size(); i++) {
		probs[i] = dirichlet_theta[i];
		if (b)
			cerr << " " << dirichlet_theta[i];
	}
	if (b)
		cerr << endl;
}

Size _Cluster::size() {
	return _num_states;
}

vector<Size>& _Cluster::getCounts(const Action& a) {
	return _outcome_counts.getValue(a);
}

Probability _Cluster::P(const Action& a, const Outcome& o) {
	Size index = _outcome_table->getOutcomeIndex(o);
	return _outcome_probs.getValue(a)[index];
}

Probability _Cluster::noModelP(const Action& a, const Outcome& o) {
	Size index = _outcome_table->getOutcomeIndex(o);
	return _outcome_probs_no_model.getValue(a)[index];
}

Probability _Cluster::logNoModelP(const State& s) {
	Probability log_p = 0;
	ActionIterator aitr(new _ActionIncrementIterator(_domain));
	while (aitr->hasNext()) {
		Action a = aitr->next();
		vector<Size>& outcome_counts = _outcome_table->getOutcomeCounts(s, a);
		for (Size outcome_index=0; outcome_index<_outcome_table->numOutcomes(); outcome_index++) {
			Outcome o = _outcome_table->getOutcome(outcome_index);
			Probability outcome_likelihood = noModelP(a, o);
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
		vector<Probability>& probs = _outcome_probs.getValue(a);
		cerr << " " << a << "(";
		for (Size i=0; i<probs.size(); i++) {
			cerr << setprecision(2) << fixed << probs[i];
			if (i != probs.size()-1)
				cerr << ",";
		}
		cerr << ")";
	}
}

_ClusterMDP::_ClusterMDP(const Domain& domain, vector<Outcome> outcomes, vector<Cluster>& cluster_vec, _FStateTable<Index>& cluster_indices)
: _domain(domain), _outcomes(outcomes), _clusters(domain), _T_map(domain) {
	vector<Cluster> cluster_copies;
	for (Size i=0; i<cluster_vec.size(); i++) {
		Cluster c(new _Cluster(*cluster_vec[i]));
		cluster_copies.push_back(c);
	}
	StateIterator sitr(new _StateIncrementIterator(_domain));
	while (sitr->hasNext()) {
		State s = sitr->next();
		Index cluster_index = cluster_indices.getValue(s);
		if (cluster_index != -1) {
			Cluster c = cluster_copies[cluster_index];
			_clusters.setValue(s, c);
		}
	}
}

StateIterator _ClusterMDP::S() {
	StateIterator sitr(new _StateIncrementIterator(_domain));
	return sitr;
}

StateIterator _ClusterMDP::predecessors(const State& s) {
	StateIterator sitr(new _StateIncrementIterator(_domain));
	return sitr;
}

ActionIterator _ClusterMDP::A() {
	ActionIterator aitr(new _ActionIncrementIterator(_domain));
	return aitr;
}

ActionIterator _ClusterMDP::A(const State& s) {
	ActionIterator aitr(new _ActionIncrementIterator(_domain));
	return aitr;
}

StateDistribution _ClusterMDP::T(const State& s, const Action& a) {
	Cluster c = _clusters.getValue(s);
	if (!c) {
		StateDistribution sd(new _EmptyStateDistribution());
		return sd;
	}
	FStateDistribution sd(new _FStateDistribution(_domain));
		
	//cerr << "building T(" << s << "," << a << ")" << endl;
	for (Size i=0; i<_outcomes.size(); i++) {
		Outcome o = _outcomes[i];
		Probability p = c->P(a, o);
		State n = o->apply(s);
		Size n_index = n.getIndex();
		n_index += 1;
		//cerr << " " << n << " : " << p << endl;
		Probability leftover = sd->P(n);
		sd->setP(n, p+leftover);
	}
	return sd;
}

Reward _ClusterMDP::R(const State& s, const Action& a) {
	Reward total = _reward_totals->getValue(s, a);
	Size count = _sa_counter->getCount(s, a);
	if (count == 0)
		return 0;
	return total/count;
}

void _ClusterMDP::printXML(std::ostream& os) {
	Cluster c = _clusters.getValue(State(_domain, 2));
	ActionIterator aitr(new _ActionIncrementIterator(_domain));
	while (aitr->hasNext()) {
		Action a = aitr->next();
		c->calcProbs(a);
	}
	_MDP::printXML(os);
}



