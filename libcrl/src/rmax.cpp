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

#include <iostream>
#include "crl/rmax.hpp"

using namespace std;
using namespace crl;

_FKnownClassifier::_FKnownClassifier(const Domain& domain, Size m)
: _counter(new _FCounter(domain)), _m(m) {
	
}

bool _FKnownClassifier::isKnown(const State& s, const Action& a) {
	Size count = _counter->getCount(s, a);
//	cout << s << ", " << a << " : " << count << endl;
	return count >= _m;
}

bool _FKnownClassifier::observe(const State& s, const Action& a, const Observation& o) {
	_counter->observe(s, a, o);
	return isKnown(s, a);
}

_RMaxMDPLearner::_RMaxMDPLearner(
  const MDPLearner& learner,
  const KnownClassifier& classifier,
  const ActionIterator& action_iterator,
  const Heuristic& heuristic
) 
: _learner(learner),
  _classifier(classifier),
  _action_iterator(action_iterator),
  _heuristic(heuristic),
  _empty_dist(new _EmptyStateDistribution()) {
	
}

_RMaxMDPLearner::_RMaxMDPLearner(
  const MDPLearner& learner,
  const KnownClassifier& classifier,
  const ActionIterator& action_iterator,
  Reward vmax)
: _learner(learner),
  _classifier(classifier),
  _action_iterator(action_iterator),
  _heuristic(new _FlatHeuristic(vmax)),
  _empty_dist(new _EmptyStateDistribution()) {
	
}

StateIterator _RMaxMDPLearner::S() {
	return _learner->S();
}
StateIterator _RMaxMDPLearner::predecessors(const State& s) {
	return _learner->predecessors(s);
}
ActionIterator _RMaxMDPLearner::A() {
	_action_iterator->reset();
	return _action_iterator;
}
ActionIterator _RMaxMDPLearner::A(const State& s) {
	return A();
}
StateDistribution _RMaxMDPLearner::T(const State& s, const Action& a) {
	if (_classifier->isKnown(s, a))
		return _learner->T(s, a);
	return _empty_dist;
}
Reward _RMaxMDPLearner::R(const State& s, const Action& a) {
	if (_classifier->isKnown(s, a))
		return _learner->R(s, a);
	return _heuristic->getPotential(s, a);
}
bool _RMaxMDPLearner::observe(const State& s, const Action& a, const Observation& o) {
	bool learned_known = _classifier->observe(s, a, o);
	bool learned_model = _learner->observe(s, a, o);
	return learned_model && learned_known;
}
