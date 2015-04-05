/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#ifndef ALP_HPP_
#define ALP_HPP_

#include <iostream>
#include "crl/dbn.hpp"
#include "crl/factor_learner.hpp"

namespace crl {

/**
 * \brief A factored (linear) value function
 * Consists of locally-scoped basis functions along with their weights
 * FIXME maybe move into factor_learner.hpp
 */
class _FactoredValueFunction {
protected:
  /// \brief The (global) domain of this value function
  const Domain _domain;
  /// \brief The basis functions, each mapping (ms,ma) -> val, where ms,ma have local scope
  /// \note For the LP to be guaranteed feasible, the constant basis h(x)=1 may have to be included in this set
  std::vector<DiscreteFunction<Reward>> _basis;
  /// \brief The weight vector associated with the basis functions for value computations
  std::vector<double> _weight;
public:
  /// \brief ctor with an optional size hint for the number of basis functions to be added
  _FactoredValueFunction(const Domain& domain, Size size_hint = 0)
  : _domain(domain) {
    if(size_hint != 0) {
        _basis.reserve(size_hint);
        _weight.reserve(size_hint);
    }
  }
  /// \brief Add a basis function (with local state and action scope) to this V-fn
  void addBasisFunction(DiscreteFunction<Reward> h, double weight);
  /// \brief (Re-)define the weight associated with basis function `i'
  void setWeight(Size i, double weight);

  //
  // Evaluation
  //

  /// \brief Return value of (global) \a State js
  virtual Reward getV(const State& js) const;
  /// \brief Return value of (global) \a State s
  virtual Reward eval(const State& js) const {
    return getV(js);
  }
  /// \brief Return the best action in (global) \a State js
  /// \note Runs distributed action selection via variable elimination in the coordination graph
  virtual Action getBestAction(const State& js) const;
  /// \brief Return the best action in (global) \a State js
  /// \note Runs distributed action selection via variable elimination (given an \a elimination order) in the coordination graph
  virtual Action getBestAction(const State& js, const SizeVec& elimination_order) const;

};
typedef boost::shared_ptr<_FactoredValueFunction> FactoredValueFunction;


/**
 * A planner that uses an approximate linear program (ALP) along with a factored value function
 * \see Guestrin, Koller, Parr, and Venkatarman, 2003
 */
class _ALPPlanner : public _Planner {
protected:
  Domain _domain;
  /// \brief The factored dynamics and reward models
  FactoredMDP _fmdp;
  /// \brief The factored value function for which weights will be computed
  FactoredValueFunction _value_fn;
  /// \brief The alpha vector (state relevance weights) associated with the basis functions
  std::vector<double> _alpha;
  /// \brief Discount factor
  float _gamma;
  /// \brief Whether all basis functions have been cached
  bool _cached;
public:
  _ALPPlanner(const FactoredMDP& fmdp, float gamma)
  : _domain(fmdp->getDomain()), _fmdp(fmdp), _gamma(gamma), _cached(false) { }

  /// \brief
  virtual Action getAction(const State& js) override {
    assert(_value_fn != nullptr);
    return _value_fn->getBestAction(js);
  }

  ///
  /// \brief run the FactoredALP algorithm: compute weights for \a FactoredValueFunction via approximate LP
  ///
  int plan();

  ///
  /// \brief Input a factored value function that will be used for solving
  ///
  void setFactoredValueFunction(FactoredValueFunction vfn) {
    _value_fn = std::move(vfn);
  }
  /// \brief Return value function associated with this ALP
  FactoredValueFunction getFactoredValueFunction() const {
    return _value_fn;
  }
};
typedef boost::shared_ptr<_ALPPlanner> ALPPlanner;

} // namespace crl

#endif
