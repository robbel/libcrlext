/*
    Copyright 2015 Philipp Robbel

    TODO: ADD LICENSE
 */

#ifndef DBN_HPP_
#define DBN_HPP_

#include "crl/crl.hpp"
#include "crl/flat_tables.hpp"


namespace crl {

typedef _StateActionTable<ProbabilityVec> _SAFProbTable;
typedef boost::shared_ptr<_SAFProbTable> SAFProbTable;

/**
 * \brief The definition of a single DBN factor across 2 time slices (t-1 -> t), that can have its dynamics set explicitly.
 *  Includes the transition probabilities encoded with tabular storage.
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

  /// \brief A mapping from (s,a) of the subdomain -> Pr(t), the target value of this factor
  SAFProbTable _prob_table;
  /// \brief A dummy, empty state
  const State _empty_s;

  ///
  /// \brief Extract the relevant state information for this factor (i.e., those corresponding to this \a _subdomain)
  /// \param s The current (complete) state
  /// \param n The (complete) successor state (e.g., from an \a Observation)
  /// \note Only available after call to \a pack()
  ///
  State mapState(const State& s, const State& n) const;
  ///
  /// \brief Extract the relevant action information for this factor (i.e., those corresponding to this \a _subdomain)
  /// \param a The (complete) joint action
  /// \note Only available after call to \a pack()
  ///
  Action mapAction(const Action& a) const;
public:
  /**
   * \brief Initialize this \a DBNFactor for a specific factor in the domain.
   */
  _DBNFactor(const Domain& domain, Size target);
  virtual ~_DBNFactor() { }
  virtual void addDelayedDependency(Size index);
  virtual void addConcurrentDependency(Size index);
  virtual void addActionDependency(Size index);
  virtual bool hasConcurrentDependency() const {
    return !_concurrent_dep.empty();
  }
  ///
  /// \brief Assemble the table corresponding to the CPT estimates for this factor
  /// \note Called after all dependencies have been added
  ///
  virtual void pack();

  /// \brief The vector of probabilities for successor values associated with the tuple (s,n,a)
  virtual const ProbabilityVec& T(const State& s, const State& n, const Action& a);
  /// \brief Convenience function for the case that no concurrent dependencies exist in 2DBN
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
 * \brief The 2-stage DBN (2DBN) encoding the transition function from t-1 to t.
 * The 2DBN supports factored states and actions.
 */
class _DBN {
protected:
  std::vector<DBNFactor> _dbn_factors;
  /// \brief whether there are any concurrent dependencies in the DBN (at time slice t)
  bool _has_concurrency;
public:
  _DBN()
  : _has_concurrency(false) { }
  virtual ~_DBN() { }

  virtual void addDBNFactor(DBNFactor dbn_factor);
  /// \brief Iterator over all \a DBNFactor in this DBN
  virtual FactorIterator factors() {
    return boost::make_shared<_FactorVecIterator>(_dbn_factors);
  }

  /// \brief True iff there are any concurrent dependencies in the DBN (at time slice t)
  virtual bool hasConcurrentDependency() const {
    return _has_concurrency;
  }
};
typedef boost::shared_ptr<_DBN> DBN;

}

#endif /*DBN_HPP_*/
