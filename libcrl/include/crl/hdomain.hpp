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
 
#ifndef HDOMAIN_HPP_
#define HDOMAIN_HPP_


#include <iostream>
#include <vector>
#include <limits>
#include <boost/shared_ptr.hpp>
#include <cpputil.hpp>
#include "crl/crl.hpp"


#if defined(WIN32) || defined(WIN64)
//the windows version of the setup for hash_maps
#include <hash_map>
#define std_hash_map stdext::hash_map
template <class K, class V>
std_hash_map<K,V> alloc_hash_map(size_t num_buckets=100) {
	return std_hash_map<K,V>();
}

#else
//the linux version of the setup for hash_maps
#include <ext/hash_map>
#define std_hash_map __gnu_cxx::hash_map
template <class K, class V>
std_hash_map<K,V> alloc_hash_map(size_t num_buckets=100) {
	return std_hash_map<K,V>(num_buckets);
}

#endif
 

namespace crl {

/**
 * A table that stores its data in a hash map
 */
template <class T>
class _HashTable {
	typedef std_hash_map<Size,T> hm_t;
protected:
	hm_t _values;
public:
	_HashTable(Size num_buckets=100)
	: _values(alloc_hash_map<Size,T>(num_buckets)) {
		
	}
	virtual ~_HashTable() { }
	virtual T& getValue(const RLType& r) {
		return _values[r.getIndex()];
	}
	virtual void setValue(const RLType& r, T t) {
		_values[r.getIndex()] = t;
	}
};

/**
 * A state/action pair table that uses a hash map as
 * its underlying data structure
 */
template <class T>
class _HStateActionTable : public _StateActionTable<T> {
	typedef std_hash_map<Size,std::vector<T> > hm_t;
protected:
	const Domain _domain;
	hm_t _sa_values;
	T _initial;
public:
	_HStateActionTable(const Domain& domain, Size num_buckets=1000)
	: _domain(domain), _sa_values(alloc_hash_map<Size,std::vector<T> >(num_buckets)) { }
	_HStateActionTable(const Domain& domain, T initial, Size num_buckets=1000)
	: _domain(domain), _sa_values(alloc_hash_map<Size,std::vector<T> >(num_buckets)), _initial(initial) { }
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
	std_hash_map<Size,Action> _best_actions;
	std_hash_map<Size,Reward> _best_qs;
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

}

#endif /*HDOMAIN_HPP_*/
