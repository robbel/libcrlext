/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <iostream>
#include "crl/dbn.hpp"

using namespace std;
using namespace crl;

//
// DBNFactor implementation
//

State _DBNFactor::mapState(const State& s, const State& n) const {
	assert(_packed);
	if(s.size() == _delayed_dep.size() && !n) // under these conditions no reduction to local scope performed
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
	if(a.size() == _action_dep.size()) // under this condition no reduction to local scope performed
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

//
// LRF implementation
//

void _LRF::pack() {
  assert(!hasConcurrentDependency());
  _subdomain = boost::make_shared<_Domain>();
  const RangeVec& state_ranges = _domain->getStateRanges();
  const RangeVec& action_ranges = _domain->getActionRanges();
  const StrVec& state_names = _domain->getStateNames();
  const StrVec& action_names = _domain->getActionNames();
  for (Size i=0; i<_delayed_dep.size(); i++) {
          Size j = _delayed_dep[i];
          _subdomain->addStateFactor(state_ranges[j].getMin(), state_ranges[j].getMax(), state_names[j]);
  }
  for (Size i=0; i<_action_dep.size(); i++) {
          Size j = _action_dep[i];
          _subdomain->addActionFactor(action_ranges[j].getMin(), action_ranges[j].getMax(), action_names[j]);
  }

  _R_map = boost::make_shared<_FStateActionTable<Reward>>(_subdomain);
  _packed = true;
}

Reward _LRF::R(const State &s, const Action &a) const {
  State ms = mapState(s, _empty_s);
  Action ma = mapAction(a);
  return _R_map->getValue(s, a);
}


void _LRF::setR(const State& s, const Action& a, Reward r) {
   State ms = mapState(s, _empty_s);
   Action ma = mapAction(a);
   // check if value in valid reward range
   _domain->getRewardRange().checkThrow(r);
   // FIXME: currently no maintaining of _known_states, _known_actions (as in _FMDP)
   _R_map->setValue(ms, ma, r);
}

//
// DBN implementation
//

void _DBN::addDBNFactor(DBNFactor dbn_factor) {
  if(dbn_factor->hasConcurrentDependency()) {
    _has_concurrency = true;
  }

  _dbn_factors.push_back(std::move(dbn_factor));
  //check deps, reorder?
}

Probability _DBN::T(const State& js, const Action& ja, const State& jn) {
  Probability p = 1.;

  _FactorVecIterator fitr(_dbn_factors);
  Size fidx = 0;
  while(fitr.hasNext()) {
      const DBNFactor& f = fitr.next();
      Factor t = jn.getFactor(fidx++); // the target value of that factor in jn
      Factor offset = t - f->getTargetRange().getMin();
      const ProbabilityVec& pv = f->T(js, jn, ja);
      p *= pv[offset];
  }

  return p;
}
