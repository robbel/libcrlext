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

#ifndef FACTOR_LEARNER_HPP_
#define FACTOR_LEARNER_HPP_

#include <boost/shared_ptr.hpp>
#include "crl/common.hpp"
#include "crl/crl.hpp"
#include "crl/flat_tables.hpp"

namespace crl {

typedef _StateActionTable<SizeVec> _SAFCountTable;
typedef boost::shared_ptr<_SAFCountTable> SAFCountTable;
typedef _StateActionTable<ProbabilityVec> _SAFProbTable;
typedef boost::shared_ptr<_SAFProbTable> SAFProbTable;

/**
 * \brief The \a Learner corresponding to a single factor.
 */
class _FactorLearner : public _Learner {
protected:
	/// The domain which includes all state and action factors
	Domain _domain;
	///
	/// \brief The subdomain relevant for this state factor
	/// \note Consists of delayed and concurrent state factors, as well as action factors
	///
	Domain _subdomain;
	/// Denoting the (single) factor in the domain considered by this learner
	Size _target;
	/// The range of the (single) factor in the domain considered by this learner
	FactorRange _target_range;
	/// Denoting delayed dependencies of this factor, i.e., an edge from t to t+1 in the DBN.
	SizeVec _delayed_dep;
	/// Denoting concurrent dependencies of this factor, i.e., an edge from t+1 to t+1 in the DBN.
	SizeVec _concurrent_dep;
	/// Denoting an action dependency of this factor, i.e., an edge from t to t+1 in the DBN.
	SizeVec _action_dep;

	SACountTable _sa_count;
	SAFCountTable _sa_f_count;

	SAFProbTable _prob_table;

	///
	/// \brief Extract the relevant state information for this factor (i.e., those corresponding to this \a _subdomain)
	/// \param s The current (complete) state
	/// \param n The (complete) successor state (e.g., from an \a Observation)
	///
	State mapState(const State& s, const State& n);
	///
	/// \brief Extract the relevant action information for this factor (i.e., those corresponding to this \a _subdomain)
	/// \param a The (complete) joint action (e.g., from an \a Observation)
	///
	Action mapAction(const Action& a);
public:
	/**
	 * \brief Initialize this \a FactorLearner for a specific factor in the domain.
	 */
	_FactorLearner(const Domain& domain, Size target);
	virtual ~_FactorLearner() { }
	virtual void addDelayedDependency(Size index);
	virtual void addConcurrentDependency(Size index);
	virtual void addActionDependency(Size index);
	///
	/// \brief Assemble the tables corresponding to the CPT estimates for this factor
	/// \note Called after all dependencies have been added
	///
	virtual void pack();

	virtual bool observe(const State& s, const Action& a, const Observation& o) override;

	virtual StateDistribution augmentDistribution(StateDistribution sd, const State& s, const Action& a);
};
typedef boost::shared_ptr<_FactorLearner> FactorLearner;

/**
 * @todo
 */
class _FactorMDPLearner : public _MDPLearner {
protected:
	Domain _domain;
	std::vector<FactorLearner> _factor_learners;
public:
	_FactorMDPLearner(const Domain& domain);
	virtual ~_FactorMDPLearner() { }

	void addFactorLearner(FactorLearner& factor_learner);

	virtual StateIterator S();
	virtual StateIterator predecessors(const State& s);
	virtual ActionIterator A();
	virtual ActionIterator A(const State& s);
	virtual StateDistribution T(const State& s, const Action& a);
	virtual Reward R(const State& s, const Action& a);

	virtual bool observe(const State& s, const Action& a, const Observation& o);

};
typedef boost::shared_ptr<_FactorMDPLearner> FactorMDPLearner;

}

#endif /*FACTOR_LEARNER_HPP_*/
