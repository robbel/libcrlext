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
#include "crl/common.hpp"
#include "crl/tables.hpp"

namespace crl {

/**
 * \brief A vector table of arbitrary \a RLType
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
};

/**
 * \brief A flat table for actions, using the flat table template.
 * Also keeps track of all actions that have values.
 */
template <class T>
class _FActionTable : public _FlatTable<T>, public _ActionTable<T> {
protected:
	/// \brief actions that have associated values
	ActionSet _set_actions;
public:
	_FActionTable(const Domain& domain)
	: _FlatTable<T>(domain->getNumActions()), _set_actions(new _ActionSet()) { }
	_FActionTable(const Domain& domain, T initial)
	: _FlatTable<T>(domain->getNumActions(), initial), _set_actions(new _ActionSet()) { }

	// ActionTable interface
	virtual void setValue(const Action& a, T t) override {
		_set_actions->insert(a);
		_FlatTable<T>::setValue(a, t);
	}
	virtual T& getValue(const Action& a) override {
		return _FlatTable<T>::getValue(a);
	}
	/**
	 * Returns an iterator that goes through all actions that have
	 * set values.
	 */
	virtual ActionIterator iterator() override {
		ActionIterator itr = boost::make_shared<_ActionSetIterator>(_set_actions);
		return itr;
	}
};

/**
 * \brief A flat table for states, using the flat table template.
 * Also keeps track of all states that have values.
 */
template <class T>
class _FStateTable : public _FlatTable<T>, public _StateTable<T> {
protected:
	/// \brief states that have associated values
	StateSet _set_states;
public:
	_FStateTable(const Domain& domain)
	: _FlatTable<T>(domain->getNumStates()), _set_states(new _StateSet()) { }
	_FStateTable(const Domain& domain, T initial)
	: _FlatTable<T>(domain->getNumStates(), initial), _set_states(new _StateSet()) { }

	// StateTable interface
	virtual void setValue(const State& s, T t) override {
		_set_states->insert(s);
		_FlatTable<T>::setValue(s, t);
	}
	virtual T& getValue(const State& s) override {
		return _FlatTable<T>::getValue(s);
	}
	/**
	 * Returns an iterator that goes through all states that have
	 * set values.
	 */
	virtual StateIterator iterator() override {
		StateIterator itr = boost::make_shared<_StateSetIterator>(_set_states);
		return itr;
	}
};

/**
 * \brief A flat, 2d vector table for (s,a) pairs mapping to type T.
 */
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
};

/**
 * \brief A Q-table using vector tables.
 * Keeps track of the best action and q-value for each state.
 */
class _FQTable : public _QTable, _FStateActionTable<Reward> {
protected:
	/// \brief A cache for the best action in each state
	std::vector<Action> _best_actions;
	/// \brief A cache for the best q value in each state (i.e., the state-value function)
	std::vector<Reward> _best_qs;

	///
	/// If heuristic is available (and has not yet been computed for state s),
	/// compute row in q-table and update best action and best q value cache for s.
	///
	void checkInitial(const State& s) {
		if (!_potential || _best_actions[s.getIndex()])
			return;
		const Size index = s.getIndex();
		_ActionIncrementIterator itr(_domain);
		_best_qs[index] = -1*std::numeric_limits<double>::max();
		while (itr.hasNext()) {
			Action a = itr.next();
			Reward r = _potential->getPotential(s, a);
			if (r > _best_qs[index]) {
				_best_qs[index] = r;
				_best_actions[index] = a;
			}
			_FStateActionTable<Reward>::setValue(s, a, r);
		}
	}
public:
	/// \brief ctor to initialize all q-values to 0
	_FQTable(const Domain& domain);
	/// \brief ctor to initialize all q-values with the supplied \a Reward
	_FQTable(const Domain& domain, Reward initial);
	/// \brief ctor which supports a \a Heuristic for estimating q-values of each state
	_FQTable(const Domain& domain, Heuristic potential);

	// QTable interface
	virtual Reward getQ(const State& s, const Action& a) override {
		checkInitial(s);
		return _FStateActionTable<Reward>::getValue(s, a);
	}
	virtual void setQ(const State& s, const Action& a, Reward r) override;
	virtual Reward getV(const State& s) override {
		checkInitial(s);
		return _best_qs[s.getIndex()];
	}
	virtual Action getBestAction(const State& s) override {
		checkInitial(s);
		if (!_best_actions[s.getIndex()]) {
			//FIXME why does this return action 0 in this case?
			return Action(_domain);
		}
		return _best_actions[s.getIndex()];
	}
};
typedef boost::shared_ptr<_FQTable> FQTable;

///
/// \brief A distribution over a \a State set implemented as a flat table.
///
class _FStateDistribution : public _Distribution<State> {
	const Domain _domain;
	std::vector<Probability> prob_vec;
	/// \brief The set of states from the \a Domain that have their probability set to something other than 0
	/// FIXME Not really used...
	_StateSet _known_states;
public:
	_FStateDistribution(const Domain& domain);
	virtual StateIterator iterator() override {
		StateIterator itr = boost::make_shared<_StateSetIterator>(_known_states);
		return itr;
	}
	virtual Probability P(const State& s) override {
		return prob_vec[s.getIndex()];
	}
	virtual State sample() override;
	void setP(const State& s, Probability p);
	void clear();
};
typedef boost::shared_ptr<_FStateDistribution> FStateDistribution;

/**
 * \brief A flat MDP with tabular storage that can have its dynamics set explicitly.
 */
class _FMDP : public _MDP {
protected:
	/// \brief The domain associated with this MDP
	const Domain _domain;
	/// \brief A mapping from (s,a) -> Pr(n)
	_FStateActionTable<FStateDistribution> _T_map;
	/// \brief A mapping from (s,a) -> r
	_FStateActionTable<Reward> _R_map;
	/// \brief The set of (flat) states in this MDP
	_StateSet _known_states;
	/// \brief The set of (joint) actions in this MDP
	_ActionSet _known_actions;
	/// \brief A dummy, empty distribution for invalid (s,a) queries
	FStateDistribution _empty_T;
	/// \brief A mapping from state index to available actions in that state
	std::vector<_ActionSet> _available_vec;
	/// \brief Linking a state to its possible predecessor set (incoming states)
	_FStateTable<StateSet> _predecessors;
public:
	_FMDP(const Domain& domain);
	virtual ~_FMDP() { }
	const Domain getDomain() {return _domain;}

	// MDP interface
	virtual StateIterator S() override;
	virtual StateIterator predecessors(const State& s) override;
	virtual ActionIterator A() override;
	virtual ActionIterator A(const State& s) override;
	virtual StateDistribution T(const State& s, const Action& a) override {
		FStateDistribution& sd = _T_map.getValue(s, a);
		if (sd) return sd;
		return _empty_T;
	}
	virtual Reward R(const State& s, const Action& a) override {
		return _R_map.getValue(s, a);
	}

        ///
        /// \brief Set transition probability from (s,a) -> n
        /// \note Probabilities are not normalized inside this function
        ///
	virtual void setT(const State& s, const Action& a, const State& n, Probability p);
	/// \brief Set reward for (s,a)
	virtual void setR(const State& s, const Action& a, Reward r);
	/// \brief Clear successor state distribution for (s,a) and remove all actions from s.
	virtual void clear(const State& s, const Action& a);
	/// \todo
	virtual void clear();
};
typedef boost::shared_ptr<_FMDP> FMDP;
inline FMDP getFMDP(const MDP& mdp) {
	return boost::static_pointer_cast<_FMDP>(mdp);
}

/**
 * \brief A class that keeps track of times (state/action) pairs have been observed.
 */
class _FCounter : public _Counter {
protected:
	Domain _domain;
	SACountTable _count_sa;
	SASCountTable _count_sa_s;
public:
	_FCounter(const Domain& domain);
	virtual ~_FCounter() { }
	/**
	 * returns an iterator over all observed next states from s,a
	 */
	virtual StateIterator iterator(const State& s, const Action& a) override;
	virtual Size getCount(const State& s, const Action& a) override;
	virtual Size getCount(const State& s, const Action& a, const State& n) override;
	virtual bool observe(const State& s, const Action& a, const Observation& o) override;
};
typedef boost::shared_ptr<_FCounter> FCounter;

/**
 * An MDP learner that updates its dynamics based on experience.
 */
class _FMDPLearner : public _MDPLearner, public _FMDP {
protected:
	Counter _counter;
public:
	_FMDPLearner(const Domain& domain);
	virtual ~_FMDPLearner();
	virtual bool observe(const State& s, const Action& a, const Observation& o) override;
	virtual StateIterator S() override {return _FMDP::S();}
	virtual StateIterator predecessors(const State& s) override {return _FMDP::predecessors(s);}
	virtual ActionIterator A() override {return _FMDP::A();}
	virtual ActionIterator A(const State& s) override {return _FMDP::A(s);}
	virtual StateDistribution T(const State& s, const Action& a) override {return _FMDP::T(s, a);}
	virtual Reward R(const State& s, const Action& a) override {return _FMDP::R(s, a);}
};
typedef boost::shared_ptr<_FMDPLearner> FMDPLearner;

}

#endif /*Domain_H_*/
