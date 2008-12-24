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
 
#ifndef Domain_H_
#define Domain_H_

#include <iostream>
#include <vector>
#include <limits>
#include <boost/shared_ptr.hpp>
#include <cpputil.hpp>
#include "crl/crl.hpp"

namespace crl {

/**
 * A template for a vector table
 */
template <class T>
class _FlatTable {
protected:
	const Size _size;
	std::vector<T> _values;
public:
	_FlatTable(Size size)
	: _size(size), _values(_size) {
		
	}
	_FlatTable(Size size, T initial)
	: _size(size), _values(_size, initial) {
		
	}
	virtual ~_FlatTable() { }
	virtual T& getValue(const RLType& r) {
		return _values[r.getIndex()];
	}
	virtual void setValue(const RLType& r, T t) {
		_values[r.getIndex()] = t;
	}
	virtual void fill(T t) {
		_values = std::vector<T>(_size, t);
	}
};

/**
 * A flat table for actions, using the flat table template.
 * Also keeps track of all actions that have values.
 */
template <class T>
class _FActionTable : public _FlatTable<T>, public _ActionTable<T> {
protected:
	ActionSet _set_actions;
public:
	_FActionTable(const Domain& domain)
	: _FlatTable<T>(domain->getNumActions()), _set_actions(new _ActionSet()) { }
	_FActionTable(const Domain& domain, T initial)
	: _FlatTable<T>(domain->getNumActions(), initial), _set_actions(new _ActionSet()) { }
	virtual void setValue(const Action& a, T t) {
		_set_actions->insert(a);
		_FlatTable<T>::setValue(a, t);
	}
	virtual T& getValue(const Action& a) {
		return _FlatTable<T>::getValue(a);
	}
	/**
	 * Returns an iterator that goes through all actions that have
	 * set values.
	 */
	virtual ActionIterator iterator() {
		ActionIterator itr(new _ActionSetIterator(_set_actions));
		return itr;
	}
	virtual void fill(T t) {
		_FlatTable<T>::fill(t);
	}
};

/**
 * A flat table for states, using the flat table template.
 * Also keeps track of all states that have values.
 */
template <class T>
class _FStateTable : public _FlatTable<T>, public _StateTable<T> {
protected:
	StateSet _set_states;
public:
	_FStateTable(const Domain& domain)
	: _FlatTable<T>(domain->getNumStates()), _set_states(new _StateSet()) { }
	_FStateTable(const Domain& domain, T initial)
	: _FlatTable<T>(domain->getNumStates(), initial), _set_states(new _StateSet()) { }
	virtual void setValue(const State& s, T t) {
		_set_states->insert(s);
		_FlatTable<T>::setValue(s, t);
	}
	virtual T& getValue(const State& s) {
		return _FlatTable<T>::getValue(s);
	}
	/**
	 * Returns an iterator that goes through all states that have
	 * set values.
	 */
	virtual StateIterator iterator() {
		StateIterator itr(new _StateSetIterator(_set_states));
		return itr;
	}
	virtual void fill(T t) {
		_FlatTable<T>::fill(t);
	}
};

template <class T>
class _FStateActionTable : public _StateActionTable<T> {
protected:
	const Domain _domain;
	std::vector<std::vector<T> > _sa_values;
public:
	_FStateActionTable(const Domain& domain)
	: _domain(domain), _sa_values(domain->getNumStates(),
      std::vector<T>(domain->getNumActions())) { }
	_FStateActionTable(const Domain& domain, T initial)
	: _domain(domain), _sa_values(domain->getNumStates(),
	  std::vector<T>(domain->getNumActions(), initial)) { }
	virtual void clear() {
		_sa_values = std::vector<std::vector<T> >(_domain->getNumStates(), std::vector<T>(_domain->getNumActions()));	
	}
	
	virtual T& getValue(const State& s, const Action& a) {
		return _sa_values[s.getIndex()][a.getIndex()];
	}
	virtual void setValue(const State& s, const Action& a, T t) {
		_sa_values[s.getIndex()][a.getIndex()] = t;
	}
	virtual void fill(T t) {
		_sa_values = std::vector<std::vector<T> >(_domain->getNumStates(),
		               std::vector<T>(_domain->getNumActions(), t));
	}
};

/**
 * A Q-table using vector tables. Keeps track of the best
 * action and q-value for each state.
 */
class _FQTable : public _QTable, _FStateActionTable<Reward> {
protected:
	std::vector<Action> _best_actions;
	std::vector<Reward> _best_qs;
public:
	_FQTable(const Domain& domain);
	_FQTable(const Domain& domain, Reward initial);
	
	virtual Reward getQ(const State& s, const Action& a) {
		return _FStateActionTable<Reward>::getValue(s, a);
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
typedef boost::shared_ptr<_FQTable> FQTable;

class _FStateDistribution : public _Distribution<State> {
	const Domain _domain;
	std::vector<Probability> prob_vec;
	_StateSet _known_states;
public:
	_FStateDistribution(const Domain& domain);
	virtual StateIterator iterator() {
		StateIterator itr(new _StateSetIterator(_known_states));
		return itr;
	}
	virtual Probability P(const State& s) {
		return prob_vec[s.getIndex()];
	}
	virtual State sample();
	void setP(const State& s, Probability p);
	void clear();
};
typedef boost::shared_ptr<_FStateDistribution> FStateDistribution;

/**
 * A flat MDP that can have its dynamics set explicitely.
 */
class _FMDP : public _MDP {
protected: 
	const Domain _domain;
	_FStateActionTable<FStateDistribution> _T_map;
	_FStateActionTable<Reward> _R_map;
	_StateSet _known_states;
	_ActionSet _known_actions;
	FStateDistribution _empty_T;
	std::vector<_ActionSet> _available_vec;
	_FStateTable<StateSet> _predecessors;
public:
	_FMDP(const Domain& domain);
	const Domain getDomain() {return _domain;}
	virtual StateIterator S();
	virtual StateIterator predecessors(const State& s);
	virtual ActionIterator A();
	virtual ActionIterator A(const State& s);
	virtual StateDistribution T(const State& s, const Action& a) {
		FStateDistribution& sd = _T_map.getValue(s, a);
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
typedef boost::shared_ptr<_FMDP> FMDP;
inline FMDP getFMDP(const MDP& mdp) {
	return boost::shared_polymorphic_downcast<_FMDP>(mdp);
}

/**
 * A class that keeps track of times states/actions have 
 * been observed.
 */
class _FCounter : public _Learner {
protected:
	Domain _domain;
	SACountTable _count_sa;
	SASCountTable _count_sa_s;
public:
	_FCounter(const Domain& domain);
	~_FCounter() { }
	/**
	 * returns an iterator over all observed next states to s,a
	 */
	virtual StateIterator iterator(const State& s, const Action& a);
	virtual Size getCount(const State& s, const Action& a);
	virtual Size getCount(const State& s, const Action& a, const State& n);
	virtual bool observe(const State& s, const Action& a, const Observation& o);
};
typedef boost::shared_ptr<_FCounter> FCounter;

/**
 * An mdp learner that updates its dynamics based on experience.
 */
class _FMDPLearner : public _MDPLearner, public _FMDP {
protected:
	FCounter _counter;
public:
	_FMDPLearner(const Domain& domain);
	~_FMDPLearner();
	virtual bool observe(const State& s, const Action& a, const Observation& o);
	virtual StateIterator S() {return _FMDP::S();}
	virtual StateIterator predecessors(const State& s) {return _FMDP::predecessors(s);}
	virtual ActionIterator A() {return _FMDP::A();}
	virtual ActionIterator A(const State& s) {return _FMDP::A(s);}
	virtual StateDistribution T(const State& s, const Action& a) {return _FMDP::T(s, a);}
	virtual Reward R(const State& s, const Action& a) {return _FMDP::R(s, a);}
};
typedef boost::shared_ptr<_FMDPLearner> FMDPLearner;

}

#endif /*Domain_H_*/
