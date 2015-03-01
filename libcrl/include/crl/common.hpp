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

#ifndef COMMON_HPP_
#define COMMON_HPP_

#include <iostream>
#include <set>
#include <cpputil.hpp>
#include "crl/util.hpp"
#include "crl/types.hpp"

namespace crl {

/**
 * \brief Describes the problem structure (states and actions), as well as Rmax and Rmin.
 * Factored states and actions are supported (but no reward structure as per normal RL)
 * Rewards are generated in the \a Environment and are not part of this structural problem definition here
 * \note A domain has nothing to do with an MDP per se (but see, e.g., \a MDPEnvironment later)
 */
class _Domain {
protected:
	RewardRange _reward_range;
	RangeVec _state_ranges;
	RangeVec _action_ranges;
	/**
	 * The index factors used in RLType, when it is a State
	 */
	SizeVec _state_index_components;
	/**
	 * The index factors used in RLType, when it is an Action
	 */
	SizeVec _action_index_components;
	Size _num_states;
	Size _num_actions;
public:
	_Domain();
	virtual ~_Domain() { }

	RewardRange getRewardRange() const {
		return _reward_range;
	}
	void setRewardRange(Reward min, Reward max) {
		_reward_range = RewardRange(min, max);
	}
	const RangeVec& getStateRanges() const {
		return _state_ranges;
	}
	const RangeVec& getActionRanges() const {
		return _action_ranges;
	}
	const SizeVec& getStateIndexComponents() const {
		return _state_index_components;
	}
	const SizeVec& getActionIndexComponents() const {
		return _action_index_components;
	}
	Size getNumStates() {
		return _num_states;
	}
	Size getNumActions() {
		return _num_actions;
	}
	Size getNumStateFactors() {
		return _state_ranges.size();
	}
	Size getNumActionFactors() {
		return _action_ranges.size();
	}

	void addStateFactor(Factor min, Factor max);
	void addActionFactor(Factor min, Factor max);
};
typedef boost::shared_ptr<_Domain> Domain;

/**
 * A basic feature vector encapsulation type. Keeps track only of
 * an index, and the setFactor/getFactor methods know how to deal
 * with the index properly. This is so that copying states and
 * actions around will be very fast. Getting and setting features
 * happens very rarely compared to copying, so the extra overhead
 * in those functions is justified.
 */
class RLType {
protected:
	/**
	 * The range of each feature
	 */
	const RangeVec* _ranges;
	/**
	 * The multaplicative factor used to get at thesupport right part of
	 * the index for a given feature
	 */
	const SizeVec* _components;
	/**
	 * A single number that represents what the index into a
	 * multidimensional array would be if it were flattened.
	 */
	Size _index;

	RLType()
	: _ranges(0), _components(0), _index(0) { }
	RLType(const RangeVec* ranges, const SizeVec* components)
	: _ranges(ranges), _components(components), _index(0) { }
	RLType(const RangeVec* ranges, const SizeVec* components, Size index)
	: _ranges(ranges), _components(components), _index(index) { }
public:
	virtual ~RLType() { }

	virtual Size size() const {
		if (!_ranges) {
			throw NullObjectException();
		}
		return _ranges->size();
	}

	virtual void setFactor(Size index, Factor data) {
		if (!_ranges) {
			throw NullObjectException();
		}
		Factor old_factor = getFactor(index);
		Factor difference = data-old_factor;
		Size index_change = (*_components)[index]*difference;
		_index = _index+index_change;
	}
	virtual const Factor getFactor(Size index) const {
		if (!_ranges) {
			throw NullObjectException();
		}
		//shift _index until this feature is the lowest order
		Size ret = _index / (*_components)[index];
		if (index < size()-1) {
			//if it isn't the last feature, use mod to chop off the others
			ret = ret%((*_ranges)[index].getSpan()+1);
		}
		//offset the factor with the minimum
		return (Factor)ret+(*_ranges)[index].getMin();
	}

	void setIndex(int index) {
		_index = index;
	}
	const Size& getIndex() const {
		return _index;
	}
	/**
	 * typecast operator for the type Size
	 */
	operator Size() const {
		return _index;
	}
	virtual void print(std::ostream& os) const;
	bool operator<(const RLType& r) const {
		return getIndex() < r.getIndex();
	}
	bool operator==(const RLType& r) const {
		return _ranges == r._ranges && getIndex() == r.getIndex();
	}
	/**
	 * returns true if this is an actual state or action (and not a nullptr)
	 */
	operator bool() const {
		return _ranges != 0;
	}
};

///
/// \brief A state in a \a Domain (i.e., as per structure definition there)
///
class State : public RLType{
public:
	State()
	: RLType() { }
	State(const Domain& domain)
	: RLType(&(domain->getStateRanges()),
		 &(domain->getStateIndexComponents())) { }
	State(const Domain& domain, Size index)
	: RLType(&(domain->getStateRanges()),
		 &(domain->getStateIndexComponents()),
		 index) { }
	virtual ~State() { }
};

///
/// \brief An action in a \a Domain (i.e., as per structure definition there)
///
class Action : public RLType {
public:
	Action()
	: RLType() { }
	Action(const Domain& domain)
	: RLType(&(domain->getActionRanges()),
      &(domain->getActionIndexComponents())) { }
	Action(const Domain& domain, Size index)
	: RLType(&(domain->getActionRanges()),
	  &(domain->getActionIndexComponents()),
	  index) { }
	virtual ~Action() { }
};

inline std::ostream& operator<<(std::ostream& os, const State& s) {
	os << "s[";
	if (!s)
		os << ".";
	else
		s.print(os);
	os << "]";
	return os;
}

inline std::ostream& operator<<(std::ostream& os, const Action& a) {
	os << "a[";
	if (!a)
		os << ".";
	else
		a.print(os);
	os << "]";
	return os;
}

/**
 * A state iterator that increments an index to get the next state.
 */
class _StateIncrementIterator : public _StateIterator {
protected:
	const Domain _domain;
	State _last;
	Size _index;
public:
	_StateIncrementIterator(const Domain& domain)
	: _domain(domain), _last(domain), _index(0) {

	}
	const State& next() {
		_last = State(_domain, _index);
		_index++;
		return _last;
	}
	bool hasNext() {
		return _index < _domain->getNumStates();
	}
	void reset() {
		_index = 0;
	}
};
typedef boost::shared_ptr<_StateIncrementIterator> StateIncrementIterator;

/**
 * An action iterator that increments an index to get the next action.
 */
class _ActionIncrementIterator : public _ActionIterator {
protected:
	const Domain _domain;
	Action _last;
	Size _index;
public:
	_ActionIncrementIterator(const Domain& domain)
	: _domain(domain), _last(domain), _index(0) {

	}
	const Action& next() {
		_last = Action(_domain, _index);
		_index++;
		return _last;
	}
	bool hasNext() {
		return _index < _domain->getNumActions();
	}
	void reset() {
		_index = 0;
	}
};
typedef boost::shared_ptr<_ActionIncrementIterator> ActionIncrementIterator;

class _StateRandomIterator : public _StateIterator {
protected:
	const Domain _domain;
	State _last;
public:
	_StateRandomIterator(const Domain& domain)
	: _domain(domain), _last(domain) {

	}
	const State& next() {
		_last = State(_domain, random()%_domain->getNumStates());
		return _last;
	}
	bool hasNext() {
		return true;
	}
	void reset() {

	}
};
typedef boost::shared_ptr<_StateRandomIterator> StateRandomIterator;

class _ActionRandomIterator : public _ActionIterator {
protected:
	const Domain _domain;
	Action _last;
public:
	_ActionRandomIterator(const Domain& domain)
	: _domain(domain), _last(domain) {

	}
	const Action& next() {
		_last = Action(_domain, random()%_domain->getNumActions());
		return _last;
	}
	bool hasNext() {
		return true;
	}
	void reset() {

	}
};
typedef boost::shared_ptr<_ActionRandomIterator> ActionRandomIterator;



/**
 * An interface for a distribution that can be sampled from and queried.
 */
template <class T>
class _Distribution {
public:
	virtual ~_Distribution() { }

	typedef boost::shared_ptr<cpputil::Iterator<T> > Iterator;

	/**
	 * Iterate through possible outcomes
	 */
	virtual Iterator iterator() = 0;
	/**
	 * The probability mass of a particular outcome
	 */
	virtual Probability P(const T& t) = 0;
	/**
	 * sample from this distribution
	 */
	virtual T sample() = 0;
	virtual void print(std::ostream& os) {
		os << "dist[";
		Iterator itr = iterator();
		while (itr->hasNext()) {
			T t = itr->next();
			os << t << ":" << P(t);
			if (itr->hasNext())
				os << ",";
		}
		os << "]";
	}
};
typedef _Distribution<State> _StateDistribution;
typedef _Distribution<Action> _ActionDistribution;
typedef boost::shared_ptr<_StateDistribution> StateDistribution;
typedef boost::shared_ptr<_ActionDistribution> ActionDistribution;

/**
 * An empty distribution ready to go
 */
template <class T>
class _EmptyDistribution : public _Distribution<T> {
protected:
	std::set<T> _empty_set;
public:
	virtual ~_EmptyDistribution() { }
	typedef boost::shared_ptr<cpputil::Iterator<T> > Iterator;
	virtual Iterator iterator() {
		Iterator itr(new cpputil::EmptyIterator<T>());
		return itr;
	}
	virtual Probability P(const T& t) {
		return 0;
	}
	virtual T sample() {
		throw DistributionException("sampling from empty distribution");
	}
};
typedef _EmptyDistribution<State> _EmptyStateDistribution;
typedef boost::shared_ptr<_EmptyStateDistribution> EmptyStateDistribution;
typedef _EmptyDistribution<Action> _EmptyActionDistribution;
typedef boost::shared_ptr<_EmptyActionDistribution> EmptyActionDistribution;

}

#endif /*COMMON_HPP_*/
