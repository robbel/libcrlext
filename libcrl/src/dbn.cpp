/*
    Copyright 2015 Philipp Robbel

    TODO: ADD LICENSE
 */

#include <iostream>
#include "crl/dbn.hpp"

using namespace std;
using namespace crl;


State _DBNFactor::mapState(const State& s, const State& n) const {
	assert(_packed);
	if(s.size() == _delayed_dep.size() && !n) // under these conditions no reduction to local scope needed
	  return s;
	State ms(_subdomain);
	for (Size i=0; i<_delayed_dep.size(); i++) {
		Size j = _delayed_dep[i];
		ms.setFactor(i, s.getFactor(j));
	}
	// append those factors corresponding to concurrent dependencies
	for (Size i=0; i<_concurrent_dep.size(); i++) {
		Size j = _concurrent_dep[i];
		ms.setFactor(i+_delayed_dep.size(), n.getFactor(j));
	}
	return ms;
}

Action _DBNFactor::mapAction(const Action& a) const {
	assert(_packed);
	if(a.size() == _action_dep.size()) // if it has already been reduced to local scope
	  return a;
	Action ma(_subdomain);
	for (Size i=0; i<_action_dep.size(); i++) {
		Size j = _action_dep[i];
		ma.setFactor(i, a.getFactor(j));
	}
	return ma;
}

_DBNFactor::_DBNFactor(const Domain& domain, Size target)
: _domain(domain), _target(target), _packed(false) {
	_target_range = _domain->getStateRanges()[_target];
}

void _DBNFactor::addDelayedDependency(Size index) {
	_delayed_dep.push_back(index);
	_packed = false;
}

void _DBNFactor::addConcurrentDependency(Size index) {
	_concurrent_dep.push_back(index);
	_packed = false;
}

void _DBNFactor::addActionDependency(Size index) {
	_action_dep.push_back(index);
	_packed = false;
}

void _DBNFactor::pack() {
	_subdomain = boost::make_shared<_Domain>();
	const RangeVec& state_ranges = _domain->getStateRanges();
	const RangeVec& action_ranges = _domain->getActionRanges();
	const StrVec& state_names = _domain->getStateNames();
	const StrVec& action_names = _domain->getActionNames();
	for (Size i=0; i<_delayed_dep.size(); i++) {
		Size j = _delayed_dep[i];
		_subdomain->addStateFactor(state_ranges[j].getMin(), state_ranges[j].getMax(), state_names[j]);
	}
	for (Size i=0; i<_concurrent_dep.size(); i++) {
		Size j = _concurrent_dep[i];
		_subdomain->addStateFactor(state_ranges[j].getMin(), state_ranges[j].getMax(), state_names[j]);
	}
	for (Size i=0; i<_action_dep.size(); i++) {
		Size j = _action_dep[i];
		_subdomain->addActionFactor(action_ranges[j].getMin(), action_ranges[j].getMax(), action_names[j]);
	}

	_prob_table = boost::make_shared<_FStateActionTable<ProbabilityVec>>(_subdomain);
	_packed = true;
}

// FIXME may be costly to always convert from (s,n,a) to index (!)
const ProbabilityVec& _DBNFactor::T(const State& s, const State& n, const Action& a)
{
   State ms = mapState(s, n);
   Action ma = mapAction(a);
   ProbabilityVec& pv = _prob_table->getValue(ms, ma);
   return pv;
}

void _DBNFactor::setT(const State& s, const State& n, const Action& a, Factor t, Probability p) {
  State ms = mapState(s, n);
  Action ma = mapAction(a);
  ProbabilityVec& pv = _prob_table->getValue(ms, ma);
  if (pv.size() == 0)
          pv.resize(_target_range.getSpan()+1, 0);

  Factor offset = t - _target_range.getMin();
  pv[offset] = p; // not normalized
}

void _DBN::addDBNFactor(DBNFactor dbn_factor) {
  if(dbn_factor->hasConcurrentDependency()) {
    _has_concurrency = true;
  }

  _dbn_factors.push_back(std::move(dbn_factor));
  //check deps, reorder?
}
