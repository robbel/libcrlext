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

namespace crl {

//
// DBNFactor implementation
//

_DBNFactor::_DBNFactor(const Domain& domain, Size target)
: _FDiscreteFunction(domain), _target(target) {
  _target_range = _domain->getStateRanges()[_target];
}

void _DBNFactor::addDelayedDependency(Size index) {
  assert(index < _domain->getNumStateFactors());
  SizeVec::iterator it = std::lower_bound(_delayed_dep.begin(), _delayed_dep.end(), index);
  if(it == _delayed_dep.end() || *it != index) {
    _delayed_dep.insert(it, index);
    _computed = false; // invalidate subdomain
  }
}

void _DBNFactor::addConcurrentDependency(Size index) {
  assert(index < _domain->getNumStateFactors());
  SizeVec::iterator it = std::lower_bound(_concurrent_dep.begin(), _concurrent_dep.end(), index);
  if(it == _concurrent_dep.end() || *it != index) {
    _concurrent_dep.insert(it, index);
    _computed = false; // invalidate subdomain
  }
}

void _DBNFactor::addActionDependency(Size index) {
  _FDiscreteFunction::addActionFactor(index);
}

void _DBNFactor::addLiftedDependency(LiftedFactor lf) {
  _FDiscreteFunction::addLiftedFactor(std::move(lf));
}

void _DBNFactor::computeSubdomain() {
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
  for (Size i=0; i<_action_dom.size(); i++) {
      Size j = _action_dom[i];
      this->_subdomain->addActionFactor(action_ranges[j].getMin(), action_ranges[j].getMax(), action_names[j]);
  }
  // add lifted operators
  for (Size i=0; i<_lifted_dom.size(); i++) {
      FactorRange range = _lifted_dom[i]->getRange();
      this->_subdomain->addStateFactor(range.getMin(), range.getMax(), "#" + std::to_string(_lifted_dom[i]->getHash()));
  }
  _computed = true;
}

void _DBNFactor::setT(const State& s, const State& n, const Action& a, Factor t, Probability p) {
  State ms = mapState(s, n);
  Action ma = mapAction(a);
  ProbabilityVec& pv = _sa_table->getValue(ms, ma);
  if (pv.size() == 0)
          pv.resize(_target_range.getSpan()+1, 0);

  Factor offset = t - _target_range.getMin();
  pv[offset] = p; // not normalized
}

//
// LRF implementation
//

Reward _LRF::R(const State &s, const Action &a) const {
  State ms = mapState(s);
  Action ma = mapAction(a);
  return _sa_table->getValue(ms, ma);
}

void _LRF::setR(const State& s, const Action& a, Reward r) {
   State ms = mapState(s);
   Action ma = mapAction(a);
   // check if value in valid reward range FIXME reward range is checked per LRF factor here, not as intended!
   _domain->getRewardRange().checkThrow(r);
   // FIXME: currently no maintaining of _known_states, _known_actions (as in _FMDP)
   _sa_table->setValue(ms, ma, r);
}

//
// DBN implementation
//

void _DBN::addDBNFactor(DBNFactor dbn_factor) {
  if(dbn_factor->hasConcurrentDependency()) {
    _has_concurrency = true;
  }
  if(dbn_factor->hasLiftedDependency()) {
    _has_lifted = true;
  }
  // insert preserving order
  auto indirection = [](const DBNFactor& a, const DBNFactor& b) -> bool { return *a < *b; };
  std::vector<DBNFactor>::iterator it = std::lower_bound(_dbn_factors.begin(), _dbn_factors.end(), dbn_factor,
                                                         indirection);
  if(it == _dbn_factors.end() || (*it)->getTarget() != dbn_factor->getTarget()) {
    _dbn_factors.insert(it, std::move(dbn_factor));
  }
  else {
    LOG_WARN("DBN Factor not added. Factor with target variable `" << dbn_factor->getTarget() << "' already exists in DBN.");
  }
}

Probability _DBN::T(const State& js, const Action& ja, const State& jn) const {
    Probability p = 1.;

    _FactorVecIterator fitr(_dbn_factors);
    Size fidx = 0;
    while(fitr.hasNext()) {
        const DBNFactor& f = fitr.next();
        Factor t = jn.getFactor(fidx++); // the target value of that factor in jn
        p *= f->T(js, jn, ja, t);
    }
    return p;
}

std::ostream& operator<<(std::ostream &os, const _DBN& dbn) {
  Size i = 0;
  os << "DBN:" << std::endl;
  for(const DBNFactor& fa : dbn._dbn_factors) {
      os << "  [" << i << "] " << *fa << std::endl;
      i++;
  }
  return os;
}

std::ostream& operator<<(std::ostream &os, const _DBNFactor& fa) {
    os << "Target " << fa.getTarget() << " with parents { ";
    for(Size px : fa.getDelayedDependencies()) {
        os << "X_d" << px << ", ";
    }
    for(Size px : fa.getConcurrentDependencies()) {
        os << "X_c" << px << ", ";
    }
    for(Size pa : fa.getActionDependencies()) {
        os << "A" << pa << ", ";
    }
    for(auto l : fa.getLiftedDependencies()) {
        os << *l << ", ";
    }
    os << "}";
    return os;
}

} // namespace crl
