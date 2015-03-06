/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth
    Copyright 2015 Philipp Robbel

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

#ifndef HDOMAIN_HPP_
#define HDOMAIN_HPP_


#include <iostream>
#include <vector>
#include <limits>
#include <boost/shared_ptr.hpp>
#include <cpputil.hpp>
#include "crl/crl.hpp"
#include "crl/common.hpp"
#include "crl/tables.hpp"

struct SizeHash {
	inline size_t operator()(const crl::Size& s) const {
		return (size_t)s;
	}
};

#if defined(WIN32) || defined(WIN64)
//the windows version of the setup for hash_maps
#include <hash_map>
#define std_hash_map stdext::hash_map
template <class K, class V, class H>
std_hash_map<K,V,H> alloc_hash_map(size_t num_buckets=100) {
	return std_hash_map<K,V,H>();
}

#else
//the linux version of the setup for hash_maps
#include <ext/hash_map>
#define std_hash_map __gnu_cxx::hash_map
template <class K, class V, class H>
std_hash_map<K,V,H> alloc_hash_map(size_t num_buckets=100) {
	return std_hash_map<K,V,H>(num_buckets);
}

#endif


namespace crl {

/**
 * A table that stores its data in a hash map
 */
template <class T>
class _HashTable {
	typedef std_hash_map<Size,T,SizeHash> hm_t;
protected:
	hm_t _values;
public:
	_HashTable(Size num_buckets=1000)
	: _values(alloc_hash_map<Size,T,SizeHash>(num_buckets)) {

	}
	virtual ~_HashTable() { }
	virtual T& getValue(const RLType& r) {
		return _values[r.getIndex()];
	}
	virtual void setValue(const RLType& r, T t) {
		_values[r.getIndex()] = t;
	}
};

template <class T>
class _HStateTable : public _HashTable<T>, public _StateTable<T> {
	_StateSet _states;
public:
	virtual T& getValue(const State& s) {
		return _HashTable<T>::getValue(s);
	}
	virtual void setValue(const State& s, T t) {
		_HashTable<T>::setValue(s, t);
	}
	virtual StateIterator iterator() {
		StateIterator sitr = boost::make_shared<_StateSetIterator>(_states);
		return sitr;
	}
};

template <class T>
class _HActionTable : public _HashTable<T>, public _ActionTable<T> {
	_ActionSet _actions;
public:
	virtual T& getValue(const Action& a) {
		return _HashTable<T>::getValue(a);
	}
	virtual void setValue(const Action& a, T t) {
		_HashTable<T>::setValue(a, t);
	}
	virtual ActionIterator iterator() {
		ActionIterator aitr = boost::make_shared<_ActionSetIterator>(_actions);
		return aitr;
	}
};

/**
 * A state/action pair table that uses a hash map as
 * its underlying data structure
 */
template <class T>
class _HStateActionTable : public _StateActionTable<T> {
	typedef std_hash_map<Size,std::vector<T>,SizeHash> hm_t;
protected:
	const Domain _domain;
	hm_t _sa_values;
	T _initial;
public:
	_HStateActionTable(const Domain& domain, Size num_buckets=1000)
	: _domain(domain), _sa_values(alloc_hash_map<Size,std::vector<T>,SizeHash>(num_buckets)) { }
	_HStateActionTable(const Domain& domain, T initial, Size num_buckets=1000)
	: _domain(domain), _sa_values(alloc_hash_map<Size,std::vector<T>,SizeHash>(num_buckets)), _initial(initial) { }
	virtual void clear() {
		_sa_values.clear();
	}

	virtual T& getValue(const State& s, const Action& a) {
		std::vector<T>& a_v = _sa_values[s.getIndex()];
		if (a_v.size() == 0)
			a_v = std::vector<T>(_domain->getNumActions(), _initial);
		return a_v[a.getIndex()];
	}
	virtual void setValue(const State& s, const Action& a, T t) {
		std::vector<T>& a_v = _sa_values[s.getIndex()];
		if (a_v.size() == 0)
			a_v = std::vector<T>(_domain->getNumActions(), _initial);
		a_v[a.getIndex()] = t;
	}
};

/**
 * A Q-table that uses a hash map as its underlying data structure.
 */
class _HQTable : public _QTable, _HStateActionTable<Reward> {
protected:
	Domain _domain;
	std_hash_map<Size,Action,SizeHash> _best_actions;
	std_hash_map<Size,Reward,SizeHash> _best_qs;
	Reward _initial;
public:
	_HQTable(const Domain& domain, Size num_buckets=1000);

	virtual Reward getQ(const State& s, const Action& a) {
		return _HStateActionTable<Reward>::getValue(s, a);
	}
	virtual void setQ(const State& s, const Action& a, Reward r);
	virtual Reward getV(const State& s) {
		return _best_qs[s.getIndex()];
	}
	virtual Action getBestAction(const State& fs) {
		if (!_best_actions[fs.getIndex()])
			return _best_actions[fs.getIndex()] = Action(_domain);
		return _best_actions[fs.getIndex()];
	}
};
typedef boost::shared_ptr<_HQTable> HQTable;

class _HStateDistribution : public _Distribution<State> {
	const Domain _domain;
	_HStateTable<Probability> _prob_hash;
	_StateSet _known_states;
public:
	_HStateDistribution(const Domain& domain);
	virtual StateIterator iterator() {
		StateIterator itr = boost::make_shared<_StateSetIterator>(_known_states);
		return itr;
	}
	virtual Probability P(const State& s) {
		return _prob_hash.getValue(s);
	}
	virtual State sample();
	void setP(const State& s, Probability p);
	void clear();
};
typedef boost::shared_ptr<_HStateDistribution> HStateDistribution;

/**
 * A class that keeps track of times states/actions have
 * been observed.
 */
class _HCounter : public _Counter {
protected:
	Domain _domain;
	SACountTable _count_sa;
	SASCountTable _count_sa_s;
public:
	_HCounter(const Domain& domain);
	~_HCounter() { }
	/**
	 * returns an iterator over all observed next states to s,a
	 */
	virtual StateIterator iterator(const State& s, const Action& a);
	virtual Size getCount(const State& s, const Action& a);
	virtual Size getCount(const State& s, const Action& a, const State& n);
	virtual bool observe(const State& s, const Action& a, const Observation& o);
};
typedef boost::shared_ptr<_HCounter> HCounter;

class _HMDP : public _MDP {
protected:
	const Domain _domain;
	_HStateActionTable<HStateDistribution> _T_map;
	_HStateActionTable<Reward> _R_map;
	_StateSet _known_states;
	_ActionSet _known_actions;
	StateDistribution _empty_T;
	_HStateTable<StateSet> _predecessors;
public:
	_HMDP(const Domain& domain);
	const Domain getDomain() {return _domain;}
	virtual StateIterator S();
	virtual StateIterator predecessors(const State& s);
	virtual ActionIterator A();
	virtual ActionIterator A(const State& s);
	virtual StateDistribution T(const State& s, const Action& a) {
		HStateDistribution& sd = _T_map.getValue(s, a);
		if (sd) return sd;
		return _empty_T;
	}
	virtual Reward R(const State& s, const Action& a) {
		return _R_map.getValue(s, a);
	}
	virtual void setT(const State& s, const Action& a, const State& n, Probability p);
	virtual void setR(const State& s, const Action& a, Reward r);
	virtual void clear(const State& s, const Action& a);
	virtual void clear();
};
typedef boost::shared_ptr<_HMDP> HMDP;
inline HMDP getHMDP(const MDP& mdp) {
	return boost::static_pointer_cast<_HMDP>(mdp);
}

/**
 * An mdp learner that updates its dynamics based on experience.
 */
class _HMDPLearner : public _MDPLearner, public _HMDP {
protected:
	HCounter _counter;
public:
	_HMDPLearner(const Domain& domain);
	virtual ~_HMDPLearner();
	virtual bool observe(const State& s, const Action& a, const Observation& o);
	virtual StateIterator S() {return _HMDP::S();}
	virtual StateIterator predecessors(const State& s) {return _HMDP::predecessors(s);}
	virtual ActionIterator A() {return _HMDP::A();}
	virtual ActionIterator A(const State& s) {return _HMDP::A(s);}
	virtual StateDistribution T(const State& s, const Action& a) {return _HMDP::T(s, a);}
	virtual Reward R(const State& s, const Action& a) {return _HMDP::R(s, a);}
};
typedef boost::shared_ptr<_HMDPLearner> HMDPLearner;


}

#endif /*HDOMAIN_HPP_*/
