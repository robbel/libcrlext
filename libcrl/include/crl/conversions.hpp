/*
    Copyright 2015 Philipp Robbel

    TODO: ADD LICENSE
 */

#ifndef CONVERSIONS_HPP_
#define CONVERSIONS_HPP_

#include <iostream>
#include <string>
#include <cassert>
#include "crl/crl.hpp"
#include "crl/flat_tables.hpp"

using namespace std;
using namespace crl;

namespace crl {

typedef _StateActionTable<ProbabilityVec> _SAFProbTable;
typedef boost::shared_ptr<_SAFProbTable> SAFProbTable;

/**
 * \brief The definition of a single DBN factor across 2 time slices (t-1 -> t).
 *  Includes the transition characteristics encoded with tabular storage.
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
  ///
  /// \brief Assemble the tables corresponding to the CPT estimates for this factor
  /// \note Called after all dependencies have been added
  ///
  virtual void pack();

};
typedef boost::shared_ptr<_DBNFactor> DBNFactor;

/**
 * \brief The 2-stage DBN encoding the transition function
 */
class _DBN {
protected:
  std::vector<DBNFactor> _dbn_factors;

public:
  _DBN() { }
  virtual ~_DBN() { }

  void addDBNFactor(DBNFactor& dbn_factor);

  // FIXME need reward function here ...  ?!

};
typedef boost::shared_ptr<_DBN> DBN;

///
/// \brief write the \a MDP out to filename in SPUDD format.
///
void exportToSpudd(MDP mdp, Domain domain, const string& filename);

}

#endif /*CONVERSIONS_HPP_*/
