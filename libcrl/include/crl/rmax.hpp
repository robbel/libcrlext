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
#ifndef RMAX_HPP_
#define RMAX_HPP_

#include <boost/shared_ptr.hpp>
#include "crl/crl.hpp"
#include "crl/fdomain.hpp"

namespace crl {

/**
 * an interface to keep track of which state/action pairs are known
 */
class _KnownClassifier : public _Learner {
public:
	virtual ~_KnownClassifier() { }
	virtual bool isKnown(const State& s, const Action& a) = 0;
	virtual bool observe(const State& s, const Action& a, const Observation& o) = 0;
};
typedef boost::shared_ptr<_KnownClassifier> KnownClassifier;

/**
 * a class to keep track of which state/action pairs are known,
 * using a flat table
 */
class _FKnownClassifier : public _KnownClassifier {
protected:
	/**
	 * counts for each s,a
	 */
	FCounter _counter;
	/**
	 * the threshold after which a pair becomes known
	 */
	Size _m;
public:
	_FKnownClassifier(const Domain& domain, Size m);
	virtual ~_FKnownClassifier() { }
	virtual bool isKnown(const State& s, const Action& a);
	virtual bool observe(const State& s, const Action& a, const Observation& o);
};
typedef boost::shared_ptr<_FKnownClassifier> FKnownClassifier;

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
