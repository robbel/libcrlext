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

void _FactorLearner::pack() {
	_DBNFactor::pack();
	// allocate remaining counters
	_sa_count = SACountTable(new _FStateActionTable<Index>(_subdomain));
	_sa_f_count = SAFCountTable(new _FStateActionTable<SizeVec>(_subdomain));
}

bool _FactorLearner::observe(const State& s, const Action& a, const Observation& o) {
	State ms = mapState(s, o->getState());
	Action ma = mapAction(a);
	// the observed value of the target factor
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

StateDistribution _FactorLearner::augmentDistribution(StateDistribution sd, const State& s, const Action& a) const {
	StateIterator itr = sd->iterator();
	
	FStateDistribution sdp(new _FStateDistribution(_domain));
	while (itr->hasNext()) {
		State n = itr->next();
		Probability p = sd->P(n); // the current probability assigned to n
		
		State ms = mapState(s, n);
		Action ma = mapAction(a);
		
		ProbabilityVec& pv = _prob_table->getValue(ms, ma);

		for (Size i=0; i<pv.size(); i++) {
			Factor f = i+_target_range.getMin();
			State np = n;
			np.setFactor(_target, f);
			Probability pp = pv[i]*p; // the updated probability assigned to n
			sdp->setP(np, pp);
		}
	}
	return sdp;
}

StateIterator _FactoredMDP::S() {
	return StateIterator();
}

StateIterator _FactoredMDP::predecessors(const State& s) {
	return StateIterator();
}

ActionIterator _FactoredMDP::A() {
	return ActionIterator();
}

ActionIterator _FactoredMDP::A(const State& s) {
	return ActionIterator();
}

StateDistribution _FactoredMDP::T(const State& s, const Action& a) {
	return StateDistribution();
}

Reward _FactoredMDP::R(const State& s, const Action& a) {
	return 0;
}

void _FactoredMDPLearner::addFactorLearner(FactorLearner factor_learner) {
	_FactoredMDP::_T_map.addDBNFactor(std::move(factor_learner));
	//check deps, reorder?
}

// note: each factor observes the (global) reward signal
bool _FactoredMDPLearner::observe(const State& s, const Action& a, const Observation& o) {
	// call observation function on each FactorLearner
	FactorIterator fitr = _FactoredMDP::_T_map.factors();
	while(fitr->hasNext()) {
	  FactorLearner f = boost::static_pointer_cast<_FactorLearner>(fitr->next()); //FIXME terrible, all these shared_ptr copies...
	  f->observe(s, a, o);
	}

	return true;
}

	
