/*
    Copyright 2009 Rutgers University
    Copyright 2009 John Asmuth
    Copyright 2015 Philipp Robbel

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

#include <cmath>
#include <iomanip>

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_sf_gamma.h>

#include <cpputil.hpp>

#include "crl/outcomes.hpp"

using namespace crl;
using namespace std;
using namespace cpputil;

vector<double> ln_gamma_cache;
inline double cached_ln_gamma(Size x) {
	if (true)
		return gsl_sf_lngamma(x);
//	cerr << "cached_ln_gamma(" << x << ")" << endl;
	while (x >= ln_gamma_cache.size()) {
		int v = ln_gamma_cache.size()+1;
		ln_gamma_cache.push_back(gsl_sf_lngamma(v));
//		cerr << "max = " << v << endl;
	}
//	cerr << "return " << ln_gamma_cache[x] << endl;
	return ln_gamma_cache[x-1];
}

_StepOutcome::_StepOutcome(Domain domain, const vector<int>& deltas, bool range_barrier)
: _domain(domain), _deltas(deltas), _range_barrier(range_barrier) {

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
		else if (!_range_barrier)
			return State(); 
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
//			cerr << s << " x " << a << " -> " << sp << " : " << o->getReward()
//			     <<  " (" << i << ")" << endl;
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
  _outcome_priors(domain, vector<Size>(_outcome_table->numOutcomes(), 1)),
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
	_log_p = 0;
}

//oh my poor code
extern int _domain_type;

_Cluster::_Cluster(const Domain& domain, OutcomeTable outcome_table, _FActionTable<std::vector<Size> > outcome_priors, gsl_rng* gsl_random)
: _domain(domain), _outcome_table(outcome_table),
  _outcome_priors(outcome_priors),
  _outcome_counts(outcome_priors),
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
	_log_p = calcLogP();
}

void _Cluster::setGSLRandom(gsl_rng* gsl_random) {
	_gsl_random = gsl_random;
}

void _Cluster::addState(const State& s) {
	_num_states++;
	_states.insert(s);
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
	
	_log_p = calcLogP();
}

void _Cluster::removeState(const State& s) {
	_num_states--;
	_states.erase(s);
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
	_log_p = calcLogP();
}

void _Cluster::calcProbs() {
	ActionIterator aitr(new _ActionIncrementIterator(_domain));
	while (aitr->hasNext()) {
		Action a = aitr->next();
		calcProbs(a);
	}
}

void _Cluster::calcProbs(const Action& a) {
	vector<Size>& cluster_action_counts = _outcome_counts.getValue(a);
	vector<Size>& priors = _outcome_priors.getValue(a);
	Size total = _outcome_totals.getValue(a)-0*priors[0]*priors.size();
	Probability totalInv = 1.0/total;
	vector<Probability>& probs_no_model = _outcome_probs_no_model.getValue(a);
	for (Size i=0; i<probs_no_model.size(); i++)
		probs_no_model[i] = totalInv*(cluster_action_counts[i]-0*priors[i]);
	
	Size K = cluster_action_counts.size();
	double* dirichlet_alpha = new double[K];
	double* dirichlet_theta = new double[K];
	for (Size i=0; i<K; i++) {
		dirichlet_alpha[i] = cluster_action_counts[i];
	}
	gsl_ran_dirichlet(_gsl_random, K, dirichlet_alpha, dirichlet_theta);
	vector<Probability>& probs = _outcome_probs.getValue(a);
	Probability sum = 0;
	for (Size i=0; i<probs.size(); i++) {
		probs[i] = dirichlet_theta[i];
		sum += probs[i];
	}
	delete dirichlet_alpha;
	delete dirichlet_theta;
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

Probability _Cluster::P(const Action& a, Size index) {
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
			Probability likelihood = noModelP(a, o);
			Probability log_likelihood = log(likelihood);
			log_p += log_likelihood * outcome_counts[outcome_index];
			//cerr << log_p << endl;
		}
	}
	return log_p;
}

time_t total_cluster_logp = 0;
time_t count_cluster_logp = 0;

Probability _Cluster::logP() {
	return _log_p;
}
extern int _domain_type;
Probability _Cluster::calcLogP() {
	time_t start = time_in_milli();
	Probability log_p = 0;
	ActionIterator aitr = ActionIterator(new _ActionIncrementIterator(_domain));
	while (aitr->hasNext()) {
		Action a = aitr->next();
		
		Probability a_log_p = 0;
		//matlab code line 1
		//  ll = ll - sum( gammaln(prior_cnts) ) + gammaln( sum(prior_cnts) );
		Size prior_sum = 0;
		vector<Size>& priors = _outcome_priors.getValue(a);
		for (Size i=0; i<priors.size(); i++) {
			prior_sum += priors[i];
			a_log_p -= cached_ln_gamma(priors[i]);
		}
		a_log_p += cached_ln_gamma(prior_sum);
		
		StateIterator sitr = StateIterator(new _StateSetIterator(_states));
		
		//matlab code lines 2
		//  ll = ll + sum( gammaln( sum(cnts,2)+1 ) );
  		sitr->reset();
		while (sitr->hasNext()) {
			State s = sitr->next();
			if (_domain_type == 1 && s.getFactor(2) == 1)
				continue; //maze purgatory
			vector<Size>& s_counts = _outcome_table->getOutcomeCounts(s, a);
			Size s_total = 0;
			for (Size i=0; i<s_counts.size(); i++) {
				s_total += s_counts[i];
			}
			a_log_p += cached_ln_gamma(s_total+1);
		}
		
		//matlab code line 3
		//  ll = ll - sum( gammaln( cnts(:) + 1 ) );
		//cnts(:) is the flattened count of all outcomes over all states
		Probability sum_gamma_counts = 0;
  		sitr->reset();
		while (sitr->hasNext()) {
			State s = sitr->next();
			vector<Size>& s_counts = _outcome_table->getOutcomeCounts(s, a);
			for (Size i=0; i<s_counts.size(); i++) {
				sum_gamma_counts += cached_ln_gamma(s_counts[i]+1);
			}
		}
		a_log_p -= sum_gamma_counts;
		
		//matlab code line 4
  		//  ll = ll + sum( gammaln( sum(cnts,1)+prior_cnts) );
  		vector<Size>& outcome_counts = _outcome_counts.getValue(a);
  		Size total_count = 0;
		for (Size i=0; i<priors.size(); i++) {
			Size count = outcome_counts[i];
			total_count += count;
			a_log_p += cached_ln_gamma(count);
		}
		
		//matlab code line 5
		//  ll = ll - gammaln( sum(sum(cnts))+sum(prior_cnts) );
		a_log_p -= cached_ln_gamma(total_count);
		log_p += a_log_p;
	}	
	
	time_t end = time_in_milli();
	total_cluster_logp += end - start;
	count_cluster_logp++;
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

_ClusterMDP::_ClusterMDP(const Domain& domain, vector<Outcome> outcomes, vector<Cluster>& cluster_vec, _FStateTable<Index>& cluster_indices, Probability log_p)
: _domain(domain), _outcomes(outcomes), _clusters(domain),
  _cluster_vec(cluster_vec),
  _rewards(domain, domain->getRewardRange().getMin()-1), 
  _T_map(domain), _log_p(log_p) {
	vector<Cluster> cluster_copies;
	for (Size i=0; i<cluster_vec.size(); i++) {
		Cluster c(new _Cluster(*(cluster_vec[i])));
		cluster_copies.push_back(c);
		c->calcProbs();
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
	_use_Beta = false;
	_m = 0;
}

void _ClusterMDP::setRewardBeta(FStateActionRewardTable reward_Beta_alpha, FStateActionRewardTable reward_Beta_beta) {
	_reward_Beta_alpha = reward_Beta_alpha;
	_reward_Beta_beta = reward_Beta_beta;
}

void _ClusterMDP::setRewardTotals(FStateActionRewardTable reward_totals, FCounter sa_counter) {
	_reward_totals = reward_totals;
	_sa_counter = sa_counter;
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

bool _ClusterMDP::isKnown(const State& s, const Action& a) {
	Cluster c = _clusters.getValue(s);
	if (c->getTotal(a) < _m)
		return false;
	Size total = 0;
	ActionIterator aitr(new _ActionIncrementIterator(_domain));
	while (aitr->hasNext())
		total += _sa_counter->getCount(s, aitr->next());
	return total >= _m;
}


bool isPit(const State& s) {
	if (false) {
                if ((s.getFactor(0) == 6 && s.getFactor(1) == 1) ||
		    (s.getFactor(0) == 0 && s.getFactor(1) == 2) ||
                    (s.getFactor(0) == 4 && s.getFactor(1) == 2) ||
		    (s.getFactor(0) == 0 && s.getFactor(1) == 3) ||
		    (s.getFactor(0) == 2 && s.getFactor(1) == 4) ||
		    (s.getFactor(0) == 0 && s.getFactor(1) == 5) ||
		    (s.getFactor(0) == 8 && s.getFactor(1) == 5) ||
		    (s.getFactor(0) == 4 && s.getFactor(1) == 6) ||
		    (s.getFactor(0) == 1 && s.getFactor(1) == 8) ||
		    (s.getFactor(0) == 3 && s.getFactor(1) == 8) ||
		    (s.getFactor(0) == 6 && s.getFactor(1) == 8))
			return true;
	}
	if (false) {
		if (s.getFactor(0) == 2 && s.getFactor(1) == 3)
			return true;
	}
	if (true) {
                if ((s.getFactor(0) == 1 && s.getFactor(1) == 1) ||
		    (s.getFactor(0) == 4 && s.getFactor(1) == 1) ||
		    (s.getFactor(0) == 4 && s.getFactor(1) == 2) ||
		    (s.getFactor(0) == 3 && s.getFactor(1) == 3))
			return true;
	}
	return false;	
}
bool isGoal(const State& s) {
	if (false) {
		if (s.getFactor(0) == 8 && s.getFactor(1) == 8)
			return true;
	}
	if (false) {
		if (s.getFactor(0) == 3 && s.getFactor(1) == 3)
			return true;
	}
	if (true) {
		if (s.getFactor(0) == 5 && s.getFactor(1) == 5)
			return true;
	}
	return false;	
}

StateDistribution _ClusterMDP::T(const State& s, const Action& a) {
	Cluster c = _clusters.getValue(s);
	bool term = true;
	if (_domain_type == 2) {
		for (Size i=0; i<s.size(); i++) {
			if (s.getFactor(i) != _domain->getStateRanges()[i].getMax())
				term = false;
		}
	}
	
	if (_domain_type == 1) {
		term = (isPit(s) || isGoal(s));
	}
	
	if (_domain_type == 0)
		term = false;
	
	if (term || !c || !isKnown(s, a)) {
		StateDistribution sd(new _EmptyStateDistribution());
		return sd;
	}
	
	FStateDistribution sd = _T_map.getValue(s, a);
	if (sd) return sd;
	
	sd = FStateDistribution(new _FStateDistribution(_domain));
	
	//cerr << "building T(" << s << "," << a << ")" << endl;
	for (Size i=0; i<_outcomes.size(); i++) {
		Outcome o = _outcomes[i];
		State n = o->apply(s);
		if (!n)
			continue;
		Probability p = c->P(a, o);
		if (_m != 0)
			p = c->noModelP(a, o);
		Size n_index = n.getIndex();
		n_index += 1;
		//cerr << " " << n << " : " << p << endl;
		Probability leftover = sd->P(n);
		sd->setP(n, p+leftover);
	}
	
	_T_map.setValue(s, a, sd);
	return sd;
}


Reward _ClusterMDP::R(const State& s, const Action& a) {
	if (_domain_type == 2) {
		if (!isKnown(s, a)) {
			return 0;
		}
		for (Size i=0; i<s.size(); i++) {
			if (s.getFactor(i) != _domain->getStateRanges()[i].getMax())
				return -1;
		}
		return 0;
	}
	else if (_domain_type == 1) {
		if (isPit(s))
			return -1;
		if (isGoal(s))
			return 1;
		return -.001;
	}
	else if (_domain_type == 0) {
		if (s.getFactor(0) == 0)
			return 2;
		if (s.getFactor(0) == _domain->getStateRanges()[0].getMax())
			return 10;
		return 0;
	}
	else {
	
		Reward r = _rewards.getValue(s, a);
		if (r == _domain->getRewardRange().getMin()-1) {
			if (_use_Beta) {
				Reward alpha = _reward_Beta_alpha->getValue(s, a);
				Reward beta = _reward_Beta_beta->getValue(s, a);
				r = gsl_ran_beta(_gsl_random, alpha, beta);
				r *= _domain->getRewardRange().getMax()-_domain->getRewardRange().getMin();
				r += _domain->getRewardRange().getMin();
				_rewards.setValue(s, a, r);
			}
			else {
				r = _reward_totals->getValue(s, a);
				Size count = _sa_counter->getCount(s, a);
				if (count == 0)
					r = _domain->getRewardRange().getMax();
				else
					r /= count;
				_rewards.setValue(s, a, r);
			}
	//		cerr << alpha << ", " << beta << " -> " << r << endl;
		}
		return r;
	
	}
}

void _ClusterMDP::printXML(std::ostream& os) {
	Cluster c = _clusters.getValue(State(_domain, 2));
	ActionIterator aitr(new _ActionIncrementIterator(_domain));
	while (aitr->hasNext()) {
		Action a = aitr->next();
		//c->calcProbs(a);
	}
	_MDP::printXML(os);
}


Probability _ClusterMDP::logP() {
	return _log_p;
}

