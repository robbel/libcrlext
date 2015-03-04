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

	/// Collect the number of times (s,a) has been observed
	SACountTable _sa_count;
	/// Collect the number of times a specific factor value has been observed after (s,a)
	SAFCountTable _sa_f_count;
	/// The likelihood of each factor value given the counts computed above
	SAFProbTable _prob_table;

	///
	/// \brief Extract the relevant state information for this factor (i.e., those corresponding to this \a _subdomain)
	/// \param s The current (complete) state
	/// \param n The (complete) successor state (e.g., from an \a Observation)
	///
	State mapState(const State& s, const State& n) const;
	///
	/// \brief Extract the relevant action information for this factor (i.e., those corresponding to this \a _subdomain)
	/// \param a The (complete) joint action
	///
	Action mapAction(const Action& a) const;
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

	///
	/// \todo
	/// \param sd The current \a StateDistribution over (TODO!)
	/// \param s The current (complete) state
	/// \param a The complete (joint) action
	/// \return The updated \a StateDistribution (Note: over the entire domain, not just this factor's subdomain!)
	///
	virtual StateDistribution augmentDistribution(StateDistribution sd, const State& s, const Action& a) const;
};
typedef boost::shared_ptr<_FactorLearner> FactorLearner;

/**
 * \brief Encapsulates a \a FactorLearner for each factor in the domain.
 * \todo implement fully...
 */
class _FactorMDPLearner : public _MDPLearner {
protected:
	/// The domain which includes all state and action factors
	Domain _domain;
	/// The set of \a FactorLearner (for each factor) comprising this learner
	std::vector<FactorLearner> _factor_learners;
public:
	_FactorMDPLearner(const Domain& domain);
	virtual ~_FactorMDPLearner() { }

	void addFactorLearner(FactorLearner& factor_learner);

	virtual StateIterator S() override;
	virtual StateIterator predecessors(const State& s) override;
	virtual ActionIterator A() override;
	virtual ActionIterator A(const State& s) override;
	virtual StateDistribution T(const State& s, const Action& a) override;
	virtual Reward R(const State& s, const Action& a) override;

	virtual bool observe(const State& s, const Action& a, const Observation& o) override;

};
typedef boost::shared_ptr<_FactorMDPLearner> FactorMDPLearner;

}

#endif /*FACTOR_LEARNER_HPP_*/
