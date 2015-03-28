/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#ifndef DBN_HPP_
#define DBN_HPP_

#include <cassert>
#include "crl/crl.hpp"
#include "crl/flat_tables.hpp"


namespace crl {

typedef _StateActionTable<ProbabilityVec> _SAFProbTable;
typedef boost::shared_ptr<_SAFProbTable> SAFProbTable;
typedef _StateActionTable<Reward> _SAFRewardTable;
typedef boost::shared_ptr<_SAFRewardTable> SAFRewardTable;

/**
 * \brief The definition of a single DBN factor across 2 time slices (t-1 -> t), that can have its dynamics set explicitly.
 *  Includes the transition probabilities encoded with tabular storage.
 * \note All dependencies (delayed, concurrent, action) are sorted internally in ascending order.
 */
class _DBNFactor {
protected:
  /// The domain which includes all state and action factors
  Domain _domain;
  ///
  /// \brief The subdomain relevant for this state factor
  /// \note Consists of delayed and concurrent state factors, as well as action factors
  ///
  Domain _subdomain;
  /// Denoting the (single) factor in the domain considered by this DBNFactor
  Size _target;
  /// The range of the (single) factor in the domain considered by this DBNFactor
  FactorRange _target_range;
  /// Denoting delayed dependencies of this factor, i.e., an edge from t to t+1 in the DBN.
  SizeVec _delayed_dep;
  /// Denoting concurrent dependencies of this factor, i.e., an edge from t+1 to t+1 in the DBN.
  SizeVec _concurrent_dep;
  /// Denoting an action dependency of this factor, i.e., an edge from t to t+1 in the DBN.
  SizeVec _action_dep;

  /// \brief A mapping from (s,a) of the subdomain -> Pr(t), the target value of this factor
  SAFProbTable _prob_table;

  /// \brief A dummy, empty state
  State _empty_s;
  /// \brief True iff \a pack() has been called on this factor (required for some function calls)
  bool _packed;
  /// \brief sanity check for this DBNFactor
  std::function<bool()> validator;
public:
  ///
  /// \brief Extract the relevant state information for this factor (i.e., those corresponding to this \a _subdomain)
  /// \param s The current (joint) state
  /// \param n The (joint) successor state (e.g., from an \a Observation)
  /// \note Only available after call to \a pack()
  /// \warning When the size of state s corresponds to size of local state space, s is directly returned (optimization)
  ///
  State mapState(const State& s, const State& n) const;
  ///
  /// \brief Extract the relevant action information for this factor (i.e., those corresponding to this \a _subdomain)
  /// \param a The (complete) joint action
  /// \note Only available after call to \a pack()
  /// \warning When the size of action a corresponds to size of local action space, a is directly returned (optimization)
  ///
  Action mapAction(const Action& a) const;

  /**
   * \brief Initialize this \a DBNFactor for a specific factor in the domain.
   */
  _DBNFactor(const Domain& domain, Size target);
  virtual ~_DBNFactor() { }
  virtual void addDelayedDependency(Size index);
  virtual const SizeVec& getDelayedDependencies() const { return _delayed_dep; }
  virtual void addConcurrentDependency(Size index);
  virtual const SizeVec& getConcurrentDependencies() const { return _concurrent_dep; }
  virtual void addActionDependency(Size index);
  virtual const SizeVec& getActionDependencies() const { return _action_dep; }
  virtual bool hasConcurrentDependency() const { return !_concurrent_dep.empty(); }

  ///
  /// \brief Assemble the table corresponding to the CPT estimates for this factor
  /// \note Called after all dependencies have been added
  ///
  virtual void pack();
  ///
  /// \brief Return subdomain associated with this factor
  /// \note Only available after call to \a pack()
  ///
  virtual Domain getSubdomain() const {
    assert(_packed);
    return _subdomain;
  }
  /// \brief Return the (number referring to the) factor in the domain that this DBNFactor represents
  virtual Size getTarget() const {
    return _target;
  }
  /// \brief Return range of this factor
  virtual const FactorRange& getTargetRange() const {
    return _target_range;
  }
  /// \brief Less-than operator overload to support sorting
  bool operator<(const _DBNFactor &rhs) const {
    return _target < rhs._target;
  }

  /// \brief The vector of probabilities for successor values associated with the tuple (s,n,a)
  virtual const ProbabilityVec& T(const State& s, const State& n, const Action& a);
  /// \brief Convenience function for the case that no concurrent dependencies exist in 2DBN
  /// \note This particular function supports either joint state/action as parameters or state/action that are already at factor scope
  virtual const ProbabilityVec& T(const State& s, const Action& a) {
    if(!_concurrent_dep.empty()) {
        throw cpputil::InvalidException("Transition function is missing state(t) to compute concurrent dependencies.");
    }
    return T(s,_empty_s,a);
  }

  ///
  /// \brief Set transition probability from (s,n,a) -> t, the target value of this factor
  /// \param s The (complete) state at t-1
  /// \param n The (complete) state at t
  /// \param a The complete (joint) action
  /// \param t The value of this DBN factor
  /// \param p The probability associated with this value
  /// \note Probabilities are not normalized inside this function
  ///
  virtual void setT(const State& s, const State& n, const Action& a, Factor t, Probability p);
  /// \brief Convenience function for the case that no concurrent dependencies exist in 2DBN
  /// \note This particular function supports either joint state/action as parameters or state/action that are already at factor scope
  virtual void setT(const State& s, const Action& a, Factor t, Probability p) {
    if(!_concurrent_dep.empty()) {
        throw cpputil::InvalidException("Transition function is missing state(t) to compute concurrent dependencies.");
    }
    setT(s,_empty_s,a,t,p);
  }
};
typedef boost::shared_ptr<_DBNFactor> DBNFactor;
// iterators
typedef cpputil::Iterator<DBNFactor> _FactorIterator;
typedef boost::shared_ptr<_FactorIterator> FactorIterator;
typedef cpputil::VectorIterator<DBNFactor> _FactorVecIterator;
typedef boost::shared_ptr<_FactorIterator> FactorVecIterator;

/**
 * \brief A Local Reward Function (LRF) is just a DBN factor with reward storage.
 * Rewards are defined as the mapping (s,a) -> r where (s,a) denotes the current state and action.
 * \note Concurrent dependencies (i.e., rewards depending on successor state n) are not supported.
 * \note In the case that local LRF scope equals joint state or action scope, variable ordering (for those) needs to be equivalent.
 */
class _LRF : public _DBNFactor {
private:
  // trick to change visibility to private -- we don't currently support reward functions with (t+1) dependencies
  using _DBNFactor::addConcurrentDependency;
protected:
  /// \brief A mapping from (s,a) in the subdomain -> r
  SAFRewardTable _R_map;
public:
  _LRF(const Domain& domain)
  : _DBNFactor(domain, 0) { } // target in parent ctor is initialized arbitrarily, irrelevant for LRF
  virtual ~_LRF() { }
  ///
  /// \brief Assemble the table corresponding to the rewards
  /// \note Called after all dependencies have been added
  ///
  virtual void pack() override;

  /// \brief The reward associated with the tuple (s,a)
  /// \note This particular function supports either joint state/action as parameters or state/action that are already at factor scope
  virtual Reward R(const State& s, const Action& a) const;
  /// \brief Set the (local) reward associated with the tuple (s,a)
  /// \note This particular function supports either joint state/action as parameters or state/action that are already at factor scope
  virtual void setR(const State& s, const Action& a, Reward r);
};
typedef boost::shared_ptr<_LRF> LRF;

/**
 * \brief The 2-stage DBN (2DBN) encoding the transition function from t-1 to t.
 * The 2DBN supports factored states and actions.
 * \note DBNFactors are sorted internally in ascending order of their target variables.
 */
class _DBN {
protected:
  std::vector<DBNFactor> _dbn_factors;
  /// \brief True iff there are any concurrent dependencies in the DBN (i.e., at time slice t)
  bool _has_concurrency;
public:
  _DBN()
  : _has_concurrency(false) { }
  virtual ~_DBN() { }
  /// \brief Return the number of \a DBNFactors in this DBN
  virtual Size size() const {
    return _dbn_factors.size();
  }

  /// \brief Add a \a DBNFactor to this \a DBN
  /// \note Overwrites an existing DBNFactor with the identical target variable
  virtual void addDBNFactor(DBNFactor dbn_factor);
  /// \brief Iterator over all \a DBNFactor in this DBN
  virtual FactorIterator factors() {
    return boost::make_shared<_FactorVecIterator>(_dbn_factors);
  }
  /// \brief Return \a DBNFactor for a specific target variable
  virtual DBNFactor factor(Size t) const {
    assert(t < size());
    return _dbn_factors[t];
  }
  /// \brief Compute the probability of transitioning from (joint) s -> n under (joint) \a Action a
  virtual Probability T(const State& js, const Action& ja, const State& jn);

  /// \brief True iff there are any concurrent dependencies in the DBN (at time slice t)
  virtual bool hasConcurrentDependency() const {
    return _has_concurrency;
  }
};
typedef boost::shared_ptr<_DBN> DBN;

}

#endif /*DBN_HPP_*/
