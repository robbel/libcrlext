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
#include "crl/fdomain.hpp"

using namespace std;
using namespace crl;


_FQTable::_FQTable(const Domain& domain)
: _FStateActionTable<Reward>(domain), _best_actions(_domain->getNumStates()),
  _best_qs(_domain->getNumStates(), -1*std::numeric_limits<float>::max()) {

}
_FQTable::_FQTable(const Domain& domain, Reward initial)
: _FStateActionTable<Reward>(domain, initial), _best_actions(_domain->getNumStates()),
  _best_qs(_domain->getNumStates(), initial) {

}

/*
	values[a_index] = v;
	Reward value = getItem(best_action);
	if (a_index == best_action.getIndex()) {
		for (crl::FAction ai(domain); !ai.end(); ai.increment()) {
			Reward q_value = getItem(ai);
			if (q_value > value) {
				best_action = ai;
			}
		}
	}
	
	if (v > value) {
		best_action.setIndex(a_index);
	}
 */
void _FQTable::setQ(const State& s, const Action& a, Reward r) {
	Size index = s.getIndex();
	_FStateActionTable<Reward>::setValue(s, a, r);
	if (!_best_actions[index]) {
		_best_actions[index] = Action(_domain, 0);
	}
	if (_best_actions[index] == a) {
		_best_qs[index] = r;
		_ActionIncrementIterator itr(_domain);
		while (itr.hasNext()) {
			Action fa = itr.next();
			Reward q = getQ(s, fa);
			if (q > _best_qs[index]) {
				_best_qs[index] = q;
				_best_actions[index] = fa;
			} 
		}
	}
	if (r > _best_qs[index]) {
		_best_qs[index] = r;
		_best_actions[index] = a;
	}
}

_FStateDistribution::_FStateDistribution(const Domain& domain)
: _domain(domain), prob_vec(_domain->getNumStates(), 0) {

}

State _FStateDistribution::sample() {
	Probability c = 0;
	Probability i = cpputil::randDouble();
	Iterator itr = iterator();
	while (itr->hasNext()) {
		State s = itr->next();
		c += P(s);
		if (c >= i)
			return s;
	}
	throw DistributionException("exceeded sum of probabilities");
}
void _FStateDistribution::setP(const State& s, Probability p) {
	prob_vec[s.getIndex()] = p;
	if (p)
		_known_states.insert(s);
	else
		_known_states.erase(_known_states.find(s));
}
void _FStateDistribution::clear() {
	prob_vec = std::vector<Probability>(_domain->getNumStates(), 0);
	_known_states.clear();
}

_FMDP::_FMDP(const Domain& domain)
: _domain(domain), _T_map(domain), _R_map(domain, 0),
  _empty_T(new _FStateDistribution(domain)),
  _available_vec(_domain->getNumStates()), _predecessors(domain) {

}
StateIterator _FMDP::S() {
	return StateSetIterator(new _StateSetIterator(_known_states));
}
StateIterator _FMDP::predecessors(const State& s) {
	StateSet& ss = _predecessors.getValue(s);
	if (!ss) {
		EmptyStateIterator itr;
		return itr;
	}
	StateIterator itr(new _StateSetIterator(ss));
	return itr;
}
ActionIterator _FMDP::A() {
	return ActionSetIterator(new _ActionSetIterator(_known_actions));
}
ActionIterator _FMDP::A(const State& s) {
	return ActionIterator(new _ActionSetIterator(_available_vec[s.getIndex()]));
}
void _FMDP::setT(const State& s, const Action& a, const State& n, Probability p) {
	_known_states.insert(s);
	_known_actions.insert(a);
	_available_vec[s.getIndex()].insert(a);
	StateSet ss = _predecessors.getValue(n);
	if (!ss) {
		ss = StateSet(new _StateSet());
		_predecessors.setValue(n, ss);
	}
	ss->insert(s);
	FStateDistribution sd = _T_map.getValue(s, a);
	if (!sd) {
		sd = FStateDistribution(new _FStateDistribution(_domain)); 
		_T_map.setValue(s, a, sd); 
	}
	sd->setP(n, p);
}	
void _FMDP::setR(const State& s, const Action& a, Reward r) {
	_known_states.insert(s);
	_known_actions.insert(a);
	_available_vec[s.getIndex()].insert(a);
	_R_map.setValue(s, a, r);
}
void _FMDP::clear(const State& s, const Action& a) {
	FStateDistribution sd = _T_map.getValue(s, a);
	if (sd)
		sd->clear();
	_available_vec[s.getIndex()].clear();
}
void _FMDP::clear() {
	_T_map.clear();
	_available_vec.clear();
	_available_vec.resize(_domain->getNumStates());
}

_FCounter::_FCounter(const Domain& domain)
: _domain(domain), _count_sa(new _FStateActionTable<size_t>(domain)),
  _count_sa_s(new _FStateActionTable<SCountTable>(domain)) {
	
}

StateIterator _FCounter::iterator(const State& s, const Action& a) {
	SCountTable c_ns = _count_sa_s->getValue(s, a);
	if (!c_ns) {
		StateIterator itr(new _EmptyStateIterator());
		return itr;
	}
	return c_ns->iterator();
}

size_t _FCounter::getCount(const State& s, const Action& a) {
	return _count_sa->getValue(s, a);
}

size_t _FCounter::getCount(const State& s, const Action& a, const State& n) {
	SCountTable c_ns = _count_sa_s->getValue(s, a);
	if (!c_ns) {
		return 0;
	}
	size_t c_sa_s = c_ns->getValue(n);
	return c_sa_s;
}

bool _FCounter::observe(const State& s, const Action& a, const Observation& o) {
	Size c_sa = _count_sa->getValue(s, a);
	_count_sa->setValue(s, a, c_sa+1);
	
	SCountTable c_ns = _count_sa_s->getValue(s, a);
	if (!c_ns) {
		c_ns = SCountTable(new _FStateTable<size_t>(_domain));
		_count_sa_s->setValue(s, a, c_ns);
	}
	if (o->getState()) {
		size_t c_sa_s = c_ns->getValue(o->getState());
		c_ns->setValue(o->getState(), c_sa_s+1);
	}
	
	return true;
}

_FMDPLearner::_FMDPLearner(const Domain& domain)
: _FMDP(domain), _counter(new _FCounter(domain)) {
	
}

_FMDPLearner::~_FMDPLearner() {
	
}

bool _FMDPLearner::observe(const State& s, const Action& a, const Observation& o) {
	Reward total_r = R(s, a)*_counter->getCount(s, a);
	total_r += o->getReward();
	
	_counter->observe(s, a, o);
	
	size_t c_sa = _counter->getCount(s, a);
	
	Probability c_inv = 1.0/c_sa;
	
	clear(s, a);
	StateIterator itr = _counter->iterator(s, a);
	while (itr->hasNext()) {
		State n = itr->next();
		size_t c_sa_n = _counter->getCount(s, a, n);
		setT(s, a, n, c_inv*c_sa_n);
	}
	
	setR(s, a, c_inv*total_r);
	
	return true;
}
