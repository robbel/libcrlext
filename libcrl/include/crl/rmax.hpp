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
#ifndef RMAX_HPP_
#define RMAX_HPP_

#include <boost/shared_ptr.hpp>
#include "crl/crl.hpp"
#include "crl/flat_tables.hpp"
#include "crl/hash_tables.hpp"

namespace crl {

/**
 * an interface to keep track of which state/action pairs are known
 */
class _KnownClassifier : public _Learner {
protected:
        /**
         * counts for each s,a
         */
        Counter _counter;
        /**
         * the threshold after which a pair becomes known
         */
        Size _m;
public:
	_KnownClassifier(Counter counter, Size m);
	_KnownClassifier(Size m);
	virtual ~_KnownClassifier() { }
	virtual bool isKnown(const State& s, const Action& a);
	virtual bool observe(const State& s, const Action& a, const Observation& o);
};
typedef boost::shared_ptr<_KnownClassifier> KnownClassifier;

/**
 * a class to keep track of which state/action pairs are known,
 * using a flat table
 */
class _FKnownClassifier : public _KnownClassifier {
public:
	_FKnownClassifier(const Domain& domain, Size m)
	: _KnownClassifier(m) {
		_counter = boost::make_shared<_FCounter>(domain);
	}
	virtual ~_FKnownClassifier() { }
};
typedef boost::shared_ptr<_FKnownClassifier> FKnownClassifier;

/**
 * a class to keep track of which state/action pairs are known,
 * using a flat table
 */
class _HKnownClassifier : public _KnownClassifier {
public:
	_HKnownClassifier(const Domain& domain, Size m)
	: _KnownClassifier(m) {
		_counter = boost::make_shared<_HCounter>(domain);
	}
	virtual ~_HKnownClassifier() { }
};
typedef boost::shared_ptr<_HKnownClassifier> HKnownClassifier;

/**
 * An MDP that uses a heuristic for the rewards from unknown state/actions,
 * otherwise what is learned in the underlying model
 */
class _RMaxMDPLearner : public _MDPLearner {
protected:
	/**
	 * the underlying model that gets updated with observations
	 */
	MDPLearner _learner;
	KnownClassifier _classifier;
	ActionIterator _action_iterator;
	/**
	 * The heuristic for unknown pairs
	 */
	Heuristic _heuristic;
	EmptyStateDistribution _empty_dist;
public:
	_RMaxMDPLearner(const MDPLearner& learner, const KnownClassifier& classifier, const ActionIterator& action_iterator, const Heuristic& heuristic);
	_RMaxMDPLearner(const MDPLearner& learner, const KnownClassifier& classifier, const ActionIterator& action_iterator, Reward vmax);
	~_RMaxMDPLearner() { }
	virtual StateIterator S();
	virtual StateIterator predecessors(const State& s);
	virtual ActionIterator A();
	virtual ActionIterator A(const State& s);
	virtual StateDistribution T(const State& s, const Action& a);
	virtual Reward R(const State& s, const Action& a);
	virtual bool observe(const State& s, const Action& a, const Observation& o);
};
typedef boost::shared_ptr<_RMaxMDPLearner> RMaxMDPLearner;

}

#endif /*RMAX_HPP_*/
