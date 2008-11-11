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
 
#include "crl/factor_learner.hpp"
 
using namespace crl;

State _FactorLearner::mapState(const State& s, const State& n) {
	State ms(_subdomain);
	for (Size i=0; i<_delayed_dep.size(); i++) {
		Size j = _delayed_dep[i];
		ms.setFactor(i, s.getFactor(j));
	}
	for (Size i=0; i<_concurrent_dep.size(); i++) {
		Size j = _concurrent_dep[i];
		ms.setFactor(i+_delayed_dep.size(), n.getFactor(j));
	}
	return ms;
}

Action _FactorLearner::mapAction(const Action& a) {
	Action ma(_subdomain);
	for (Size i=0; i<_action_dep.size(); i++) {
		Size j = _action_dep[i];
		ma.setFactor(i, a.getFactor(j));
	}
	return ma;
}

_FactorLearner::_FactorLearner(const Domain& domain, Size target)
: _domain(domain), _target(target) {
	_target_range = _domain->getStateRanges()[_target];
}

void _FactorLearner::addDelayedDependency(Size index) {
	_delayed_dep.push_back(index);
}

void _FactorLearner::addConcurrentDependency(Size index) {
	_concurrent_dep.push_back(index);
}

void _FactorLearner::addActionDependency(Size index) {
	_action_dep.push_back(index);
}

void _FactorLearner::pack() {
	_subdomain = Domain(new _Domain());
	const RangeVec& state_ranges = _domain->getStateRanges();
	const RangeVec& action_ranges = _domain->getActionRanges();
	for (Size i=0; i<_delayed_dep.size(); i++) {
		Size j = _delayed_dep[i];
		_subdomain->addStateFactor(state_ranges[j].getMin(), state_ranges[j].getMax());
	}
	for (Size i=0; i<_concurrent_dep.size(); i++) {
		Size j = _concurrent_dep[i];
		_subdomain->addStateFactor(state_ranges[j].getMin(), state_ranges[j].getMax());
	}
	for (Size i=0; i<_action_dep.size(); i++) {
		Size j = _action_dep[i];
		_subdomain->addActionFactor(action_ranges[j].getMin(), action_ranges[j].getMax());
	}
	
	_sa_count = SACountTable(new _FStateActionTable<Size>(_subdomain));
	_sa_f_count = SAFCountTable(new _FStateActionTable<SizeVec>(_subdomain));
	_prob_table = SAFProbTable(new _FStateActionTable<ProbabilityVec>(_subdomain));
}

bool _FactorLearner::observe(const State& s, const Action& a, const Observation& o) {
	State ms = mapState(s, o->getState());
	Action ma = mapAction(a);
	Factor t = o->getState().getFactor(_target);
	Factor offset = t - _target_range.getMin();
	
	Size sa_count = _sa_count->getValue(ms, ma);
	_sa_count->setValue(ms, ma, ++sa_count);
	
	SizeVec& fv = _sa_f_count->getValue(ms, ma);
	if (fv.size() == 0)
		fv.resize(_target_range.getSpan()+1, 0);
	fv[offset] += 1;
	
	ProbabilityVec& pv = _prob_table->getValue(ms, ma);
	if (pv.size() == 0)
		pv.resize(_target_range.getSpan()+1, 0);
		
	Probability total_inv = 1.0/sa_count;
	for (Size i=0; i<fv.size(); i++)
		pv[i] = fv[i]*total_inv;
	
	return true;
}

StateDistribution _FactorLearner::augmentDistribution(StateDistribution sd, const State& s, const Action& a) {
	StateIterator itr = sd->iterator();
	
	FStateDistribution sdp(new _FStateDistribution(_domain));
	while (itr->hasNext()) {
		State n = itr->next();
		Probability p = sd->P(n);
		
		State ms = mapState(s, n);
		Action ma = mapAction(a);
		
		ProbabilityVec& pv = _prob_table->getValue(ms, ma);

		for (Size i=0; i<pv.size(); i++) {
			Factor f = i+_target_range.getMin();
			State np = n;
			np.setFactor(_target, f);
			Probability pp = pv[i]*p;
			sdp->setP(np, pp);
		}
	}
	return sdp;
}

_FactorMDPLearner::_FactorMDPLearner(const Domain& domain)
: _domain(domain) {
	
}

void _FactorMDPLearner::addFactorLearner(FactorLearner& factor_learner) {
	_factor_learners.push_back(factor_learner);
	//check deps, reorder?
} 

StateIterator _FactorMDPLearner::S() {
	return StateIterator();
}

StateIterator _FactorMDPLearner::predecessors(const State& s) {
	return StateIterator();
}

ActionIterator _FactorMDPLearner::A() {
	return ActionIterator();
}

ActionIterator _FactorMDPLearner::A(const State& s) {
	return ActionIterator();
}

StateDistribution _FactorMDPLearner::T(const State& s, const Action& a) {
	return StateDistribution();
}

Reward _FactorMDPLearner::R(const State& s, const Action& a) {
	return 0;
}

bool _FactorMDPLearner::observe(const State& s, const Action& a, const Observation& o) {
	return true;
}

	
