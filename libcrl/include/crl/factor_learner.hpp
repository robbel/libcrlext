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

#ifndef FACTOR_LEARNER_HPP_
#define FACTOR_LEARNER_HPP_

#include "crl/common.hpp"
#include "crl/crl.hpp"
#include "crl/flat_tables.hpp"
#include "crl/dbn.hpp"

namespace crl {

typedef _StateActionTable<SizeVec> _SAFCountTable;
typedef boost::shared_ptr<_SAFCountTable> SAFCountTable;
typedef _StateActionTable<ProbabilityVec> _SAFProbTable;
typedef boost::shared_ptr<_SAFProbTable> SAFProbTable;

/**
 * \brief The \a Learner corresponding to a single factor, based on observance counts.
 * Implemented with tabular storage.
 */
class _FactorLearner : public _DBNFactor, public _Learner {
protected:
	/// Collect the number of times (s,a) has been observed
	SACountTable _sa_count;
	/// Collect the number of times a specific factor value has been observed after (s,a)
	SAFCountTable _sa_f_count;
public:
	/**
	 * \brief Initialize this \a FactorLearner for a specific factor in the domain.
	 */
	_FactorLearner(const Domain& domain, Size target)
	: _DBNFactor(domain, target) { }
	virtual ~_FactorLearner() { }

	// DBNFactor interface
	virtual void pack() override;

	// Learner interface
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
 * \brief An implementation of a factored MDP with factored state and action spaces.
 * Implemented with tabular storage.
 */
class _FactoredMDP : public _MDP {
protected:
  /// \brief The domain which includes all state and action factors
  const Domain _domain;
  /// \brief The factored transition function
  _DBN _T_map;
  /// \brief The local reward functions, each mapping (ms,ma) -> r, where ms,ma have local scope
  std::vector<LRF> _lrf_factors;
public:
  _FactoredMDP(Domain domain)
  : _domain(domain) { }
  Domain getDomain() const {return _domain;}

  // MDP interface
  virtual StateIterator S() override;
  virtual StateIterator predecessors(const State& s) override;
  virtual ActionIterator A() override;
  virtual ActionIterator A(const State& s) override;
  virtual StateDistribution T(const State& s, const Action& a) override;
  virtual Probability T(const State& s, const Action& a, const State& s_next) override {
    return _T_map.T(s, a, s_next);
  }
  /// \brief return the sum of rewards accumulated over all local reward functions (LRFs) at (s,a)
  virtual Reward R(const State& s, const Action& a) override;
  /// \brief return entire factored transition function
  DBN T() const {
    DBN dbn = boost::make_shared<_DBN>(_T_map);
    return dbn;
  }

  /// \brief Add a factor to the factored transition function
  virtual void addDBNFactor(DBNFactor fac) {
    _T_map.addDBNFactor(std::move(fac));
  }
  /// \brief Add a local reward function to this factored MDP
  virtual void addLRF(LRF lrf) {
    _lrf_factors.push_back(std::move(lrf));
  }
  /// \brief Obtain all local reward functions
  const std::vector<LRF>& getLRFs() const {
    return _lrf_factors;
  }
};
typedef boost::shared_ptr<_FactoredMDP> FactoredMDP;

/**
 * \brief Encapsulates a \a FactorLearner for each factor in the domain.
 * \todo implement fully...
 */
class _FactoredMDPLearner : public _MDPLearner, public _FactoredMDP {
private:
	// trick to change visibility to private -- we don't want non-FactorLearners to be added to underlying DBN
	using _FactoredMDP::addDBNFactor;
public:
	_FactoredMDPLearner(const Domain& domain)
	: _FactoredMDP(domain) { }
	virtual ~_FactoredMDPLearner() { }

	/// \brief add a \a FactorLearner to this \a FactorMDPLearner
	virtual void addFactorLearner(FactorLearner factor_learner);

	// Learner interface
	virtual bool observe(const State& s, const Action& a, const Observation& o) override;
};
typedef boost::shared_ptr<_FactoredMDPLearner> FactorMDPLearner;

}

#endif /*FACTOR_LEARNER_HPP_*/
