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
#include "crl/hash_tables.hpp"

using namespace std;
using namespace crl;


_HStateDistribution::_HStateDistribution(const Domain& domain)
: _domain(domain), _prob_hash() {

}

State _HStateDistribution::sample() {
	Probability c = 0;
	Probability i = cpputil::randDouble();
	Iterator itr = iterator();
	while (itr->hasNext()) {
		const State& s = itr->next();
		c += P(s);
		if (c >= i)
			return s;
	}

	throw DistributionException("exceeded sum of probabilities");
}
void _HStateDistribution::setP(const State& s, Probability p) {
	if (!s) //terminal state represented by sub-1 distribution
		return;
	_prob_hash.setValue(s, p);
	if (p)
		_known_states.insert(s);
	else
		_known_states.erase(_known_states.find(s));
}
void _HStateDistribution::clear() {
	_prob_hash = _HStateTable<Probability>();
	_known_states.clear();
}



_HQTable::_HQTable(const Domain& domain, Size num_buckets)
: _HStateActionTable<Reward>(domain),
  _domain(domain),
  _best_actions(alloc_hash_map<Size,Action,SizeHash>(num_buckets)),
  _best_qs(alloc_hash_map<Size,Reward,SizeHash>(num_buckets)) {

}

void _HQTable::setQ(const State& s, const Action& a, Reward r) {
	Size index = s.getIndex();
	_HStateActionTable<Reward>::setValue(s, a, r);
	if (!_best_actions[index]) {
		_best_actions[index] = Action(_domain, 0);
	}
	if (_best_actions[index] == a) {
		_best_qs[index] = r;
		_ActionIncrementIterator itr(_domain);
		while (itr.hasNext()) {
			const Action& fa = itr.next();
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


_HCounter::_HCounter(const Domain& domain)
: _domain(domain), _count_sa(new _HStateActionTable<Index>(domain)),
  _count_sa_s(new _HStateActionTable<SCountTable>(domain)) {

}

StateIterator _HCounter::iterator(const State& s, const Action& a) {
	SCountTable c_ns = _count_sa_s->getValue(s, a);
	if (!c_ns) {
		StateIterator itr(new _EmptyStateIterator());
		return itr;
	}
	return c_ns->iterator();
}

Size _HCounter::getCount(const State& s, const Action& a) {
	return _count_sa->getValue(s, a);
}

Size _HCounter::getCount(const State& s, const Action& a, const State& n) {
	SCountTable c_ns = _count_sa_s->getValue(s, a);
	if (!c_ns) {
		return 0;
	}
	Size c_sa_s = c_ns->getValue(n);
	return c_sa_s;
}

bool _HCounter::observe(const State& s, const Action& a, const Observation& o) {
	Size c_sa = _count_sa->getValue(s, a);
	_count_sa->setValue(s, a, c_sa+1);

	SCountTable c_ns = _count_sa_s->getValue(s, a);
	if (!c_ns) {
		c_ns = SCountTable(new _HStateTable<Index>());
		_count_sa_s->setValue(s, a, c_ns);
	}
	if (o->getState()) {
		Size c_sa_s = c_ns->getValue(o->getState());
		c_ns->setValue(o->getState(), c_sa_s+1);
	}

	return true;
}


_HMDP::_HMDP(const Domain& domain)
: _domain(domain), _T_map(domain), _R_map(domain, 0, 1000),
  _empty_T(new _HStateDistribution(domain)),
  _predecessors() {

}
StateIterator _HMDP::S() {
	return StateSetIterator(new _StateSetIterator(_known_states));
}
StateIterator _HMDP::predecessors(const State& s) {
	StateSet& ss = _predecessors.getValue(s);
	if (!ss) {
		EmptyStateIterator itr;
		return itr;
	}
	StateIterator itr(new _StateSetIterator(ss));
	return itr;
}
ActionIterator _HMDP::A() {
	return ActionSetIterator(new _ActionSetIterator(_known_actions));
}
ActionIterator _HMDP::A(const State& s) {
	return A();
}
void _HMDP::setT(const State& s, const Action& a, const State& n, Probability p) {
	_known_states.insert(s);
	_known_actions.insert(a);
	StateSet ss = _predecessors.getValue(n);
	if (!ss) {
		ss = StateSet(new _StateSet());
		_predecessors.setValue(n, ss);
	}
	ss->insert(s);
	HStateDistribution sd = _T_map.getValue(s, a);
	if (!sd) {
		sd = HStateDistribution(new _HStateDistribution(_domain));
		_T_map.setValue(s, a, sd);
	}
	sd->setP(n, p);
}
void _HMDP::setR(const State& s, const Action& a, Reward r) {
	_known_states.insert(s);
	_known_actions.insert(a);
	_R_map.setValue(s, a, r);
}
void _HMDP::clear(const State& s, const Action& a) {
	HStateDistribution sd = _T_map.getValue(s, a);
	if (sd)
		sd->clear();
}
void _HMDP::clear() {
	_T_map.clear();
}


_HMDPLearner::_HMDPLearner(const Domain& domain)
: _HMDP(domain), _counter(new _HCounter(domain)) {

}

_HMDPLearner::~_HMDPLearner() {

}

bool _HMDPLearner::observe(const State& s, const Action& a, const Observation& o) {
	Reward total_r = R(s, a)*_counter->getCount(s, a);
	total_r += o->getReward();

	_counter->observe(s, a, o);

	Size c_sa = _counter->getCount(s, a);

	Probability c_inv = 1.0/c_sa;

	clear(s, a);
	StateIterator itr = _counter->iterator(s, a);
	while (itr->hasNext()) {
		const State& n = itr->next();
		Size c_sa_n = _counter->getCount(s, a, n);
		setT(s, a, n, c_inv*c_sa_n);
	}

	setR(s, a, c_inv*total_r);

	return true;
}
