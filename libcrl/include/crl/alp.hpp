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
 * Consists of locally-scoped basis functions along with their weights.
 * Additionally supports localized Q-function computations via variable elimination.
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
  /// \brief The gamma-discounted backprojections associated with the basis functions in _basis
  std::vector<DiscreteFunction<Reward>> _backprojection;
  /// \brief True iff backprojections have already been multiplied by weight parameters to allow Q function computations
  /// \see getBestAction(), getQ(), discount()
  bool _bp_discounted;
  /// \brief The reward functions to compute the local Q-functions
  /// \see getBestAction()
  std::vector<DiscreteFunction<Reward>> _lrfs;
  /// \brief Discounts the backprojections with the (computed) weights to allow Q function computations
  /// \see getBestAction(), getQ()
  void discount();
  /// \brief True iff the Backprojections inside this value functions have been multiplied by \f$\mathbf{w}\f$ already
  /// \see getBestAction()
  bool isDiscounted() const { return _bp_discounted; }
public:
  /// \brief ctor with an optional size hint for the number of basis functions to be added
  _FactoredValueFunction(const Domain& domain, Size size_hint = 0)
  : _domain(domain), _bp_discounted(false) {
    if(size_hint != 0) {
        _basis.reserve(size_hint);
        _weight.reserve(size_hint);
        _backprojection.reserve(size_hint);
    }
  }
  /// \brief Add a basis function (with local state and action scope) to this V-fn
  void addBasisFunction(DiscreteFunction<Reward> h, double weight);
  /// \brief Get the basis functions
  const std::vector<DiscreteFunction<Reward>>& getBasis() const {
      return _basis;
  }
  /// \brief (Re-)define the weight associated with basis function `i'
  void setWeight(Size i, double weight);
  /// \brief Obtain storage location for basis function weights
  std::vector<double>& getWeight() {
    return _weight;
  }
  const std::vector<double>& getWeight() const {
    return _weight;
  }
  /// \brief Adds a gamma-discounted backprojection of a basis function to this value function
  /// \note Will be modified inside this function (multiplied by weight vector during discount())
  void addBackprojection(DiscreteFunction<Reward> bp) {
      assert(_backprojection.size() < _basis.size());
      _backprojection.push_back(std::move(bp));
      _bp_discounted = false;
  }
  /// \brief Remove all backprojections
  void clearBackprojections() {
    // TODO: swap from stored location instead of recomputation
    _backprojection.clear();
    _bp_discounted = false;
  }
  /// \brief Set the reward function vector
  void setLRFs(const std::vector<DiscreteFunction<Reward>>& lrfs) {
      _lrfs = lrfs;
  }

  //
  // Evaluation
  //

  /// \brief Return value of (global) \a State js
  Reward getV(const State& js) const;
  /// \brief Return value of (global) \a State js
  Reward eval(const State& js) const {
    return getV(js);
  }
  /// \brief Return value of (global) \a State js and \a Action ja
  /// \note Exercises the Q function representation of this value function
  Reward getQ(const State& js, const Action& ja);
  /// \brief Return value of (global) \a State js and \a Action ja
  Reward eval(const State& js, const Action& ja) {
      return getQ(js,ja);
  }
  /// \brief Return the best action in (global) \a State js along with the optimum value
  /// \note Runs distributed action selection via variable elimination (default action ordering) in the coordination graph
  std::tuple<Action,Reward> getBestAction(const State& js) {
      const SizeVec elim_order = cpputil::ordered_vec<Size>(_domain->getNumActionFactors(), _domain->getNumStateFactors());
      return getBestAction(js, elim_order);
  }
  /// \brief Return the best action in (global) \a State js along with the optimum value
  /// Runs distributed action selection via variable elimination (given an \a elimination order) in the coordination graph
  /// \note The elimination order is over action factors only
  std::tuple<Action,Reward> getBestAction(const State& js, const SizeVec& elimination_order);

  /// \brief Returns the Q-function maximized over all actions
  /// Runs variable elimination (given an \a elimination order for actions)
  /// This templated version allows the specification of the type of functions returned during variable elimination
  /// \note used during (factored) Bellman Error computations
  template<class T>
  FunctionSet<Reward> getMaxQ(const SizeVec& elimination_order) {
    assert(_backprojection.size() == _basis.size());
    assert(!_lrfs.empty());

    // Discount stored backprojections once to form local Q functions
    if(!isDiscounted()) {
        discount();
    }

    // run variableElimination over joint action space
    FunctionSet<Reward> F(_domain);
    for(auto& bp : _backprojection) {
        if(bp == nullptr) {
            continue;
        }
        assert(bp->getSubdomain()->getNumStateFactors() != 0);
        F.insert(bp);
    }
    for(auto& b : _lrfs) {
        assert(b->getSubdomain()->getNumStateFactors() != 0);
        F.insert(b);
    }

    auto retFns = algorithm::variableElimination<T>(F, elimination_order);
  #if !NDEBUG
    const std::vector<DiscreteFunction<Reward>>& empty_fns = std::get<1>(retFns);
    // every term should still be a function of x
    assert(empty_fns.empty());
  #endif
    return F;
  }
  /// \brief Returns the Q-function maximized over all actions
  /// Runs variable elimination (given an \a elimination order for actions)
  /// \note used during (factored) Bellman Error computations
  FunctionSet<Reward> getMaxQ(const SizeVec& elimination_order) {
    return getMaxQ<_FDiscreteFunction<Reward>>(elimination_order);
  }
  /// \brief Returns the Q-function maximized over all actions
  /// Runs variable elimination (default action ordering)
  /// This templated version allows the specification of the type of functions returned during variable elimination
  template<class T>
  FunctionSet<Reward> getMaxQ() {
    const SizeVec elim_order = cpputil::ordered_vec<Size>(_domain->getNumActionFactors(), _domain->getNumStateFactors());
    return getMaxQ<T>(elim_order);
  }
  /// \brief Returns the Q-function maximized over all actions
  /// Runs variable elimination (default action ordering)
  FunctionSet<Reward> getMaxQ() {
    return getMaxQ<_FDiscreteFunction<Reward>>();
  }
};
typedef boost::shared_ptr<_FactoredValueFunction> FactoredValueFunction;

/// \brief A factored function consisting of (0) functionals with variable dependencies, and (1) functionals with empty scope
/// This represents a sum of locally-scoped (potentially empty-scoped) functionals
template<class T>
using FactoredFunction = std::tuple<std::vector<DiscreteFunction<T>>, std::vector<DiscreteFunction<T>>>;

//
// Additional algorithm declarations
//

namespace algorithm {
  //
  // Helper functions
  //
  /// \brief Collect basis and max-Q functionals and return modified copies (e.g., after multiplication with weight vector)
  /// This templated version supports templated versions of getMaxQ that may return other function types than _FDiscreteFunction
  /// \note Helper function for factored Bellman error and residual computations
  template<class T>
  FunctionSet<Reward> factoredBellmanFunctionals(const Domain& domain, const FactoredValueFunction& fval) {
    assert(fval->getWeight().size() == fval->getBasis().size());

    // multiply basis with w vector
    std::vector<DiscreteFunction<Reward>> modbasis;
    const auto& weight = fval->getWeight();
    std::vector<double>::size_type j = 0;
    for(const auto& h : fval->getBasis()) {
        // supports Indicator and FDiscreteFunction basis for now
        const _Indicator<>* pI = dynamic_cast<const _Indicator<>*>(h.get());
        if(pI) {
            FDiscreteFunction<Reward> wh = boost::make_shared<_FDiscreteFunction<Reward>>(domain);
            wh->join(pI->getStateFactors(), pI->getActionFactors(), pI->getLiftedFactors());
            wh->pack();
            wh->values()[pI->getStateIndex()] = weight[j++];
            modbasis.push_back(std::move(wh));
        }
        else {
            const _FDiscreteFunction<Reward>* pf = dynamic_cast<const _FDiscreteFunction<Reward>*>(h.get());
            assert(pf);
            FDiscreteFunction<Reward> wh = boost::make_shared<T>(*pf);
            (*wh) *= weight[j++];
            modbasis.push_back(std::move(wh));
        }
    }

    // run variable elimination over all action factors
    FunctionSet<Reward> q_set = fval->getMaxQ<T>();
    auto qfns = q_set.getFunctions();
    LOG_INFO("MaxQ-Function has " << qfns.size() << " local terms.");
    for(const auto& fn : qfns) {
      LOG_INFO(fn->getStateFactors().size());
    }
    // negate maxQ; maintain function type T for terms generated during variable elimination
    // this is to efficiently support `action-connected' partitioning of the maxQ-function
    std::vector<DiscreteFunction<Reward>> modqfns;
    for(const auto& qf : qfns) {
        FDiscreteFunction<Reward> nqf;
        const T* pqf = dynamic_cast<const T*>(qf.get()); // if custom type was returned
        if(pqf) {
            nqf = boost::make_shared<T>(*pqf);
        }
        else {
            const _FDiscreteFunction<Reward>* pqf = dynamic_cast<const _FDiscreteFunction<Reward>*>(qf.get());
            assert(pqf);
            nqf = boost::make_shared<_FDiscreteFunction<Reward>>(*pqf);
        }
        (*nqf) *= (-1.);
        modqfns.push_back(std::move(nqf));
    }

    FunctionSet<Reward> F(domain);
    for(auto& bp : modqfns) {
        assert(bp->getSubdomain()->getNumStateFactors() != 0);
        F.insert(std::move(bp));
    }
    for(auto& b : modbasis) {
        assert(b->getSubdomain()->getNumStateFactors() != 0);
        F.insert(std::move(b));
    }

    return F;
  }
  /// \brief Collect basis and max-Q functionals and return modified copies (e.g., after multiplication with weight vector)
  /// \note Helper function for factored Bellman error and residual computations
  FunctionSet<Reward> factoredBellmanFunctionals(const Domain& domain, const FactoredValueFunction& fval);
  /// \brief Compute coefficient \f$\lambda_k\f$ for factor \f$\phi_k\f$ in Bellman marginal function
  /// \param delVars The variables from the domain that are being summed out
  double computeMarginalLambda(const Domain& domain, const DiscreteFunction<Reward>& phi, const SizeVec& delVars);
  /// \brief Compute (relative) coefficient \f$\lambda_k\f$ for factor \f$\phi_k\f$ in Bellman marginal function
  /// A relative \f$\lambda_k\f$ is normalized w.r.t. all states in the Domain and works for `big' Domains as well
  /// \param delVars The variables from the domain that are being summed out
  /// \see _Domain::isBig()
  double computeRelativeMarginalLambda(const Domain& domain, const DiscreteFunction<Reward>& phi, const SizeVec& delVars);

  //
  // Main algorithms
  //
  /// \brief Compute expression over the (factored) Bellman residual via variable elimination (given an \a elimination_order over subset of state factors)
  /// \param op A function pointer denoting the operation to perform (e.g., max or min)
  /// \return A tuple of (0) the generated functions with variable dependencies, and (1) the generated functions with empty scope
  template<class Op = DiscreteFunction<Reward> (*)(const _DiscreteFunction<Reward>*, Size, bool)>
  FactoredFunction<Reward> factoredBellmanResidual(const Domain& domain, const FactoredValueFunction& fval, const SizeVec& elimination_order, Op op) {
    FunctionSet<Reward> F = factoredBellmanFunctionals(domain, fval);
    // run variableElimination over state factors
    SizeVec mutable_elim(elimination_order);
    auto elimTpl = algorithm::variableEliminationHeur(F, mutable_elim, ElimHeuristic::MIN_SCOPE, op);

    // store functions that retain variable dependencies
    std::vector<DiscreteFunction<Reward>> var_fns = F.getFunctions();
    return std::make_tuple(std::move(var_fns), std::move(std::get<1>(elimTpl)));
  }
  /// \brief Compute Bellman marginal, i.e., sum out all variables from residual except those in \a vars
  /// \param vars The variables spanning the domain of the returned marginal functions
  /// \return A tuple of (0) the generated functions that depend on \a vars, and (1) the generated functions with empty scope
  FactoredFunction<Reward> factoredBellmanMarginal(const Domain& domain, const SizeVec& vars, const FactoredValueFunction& fval);
  /// \brief Compute Bellman marginal, i.e., sum out all variables from residual except those in \a vars
  /// \param vars The variables spanning the domain of the returned marginal functions
  /// \param maxQ The (factored) max-Q function associated with the value function
  /// \return A tuple of (0) the generated functions that depend on \a vars, and (1) the generated functions with empty scope
  FactoredFunction<Reward> factoredBellmanMarginal(const Domain& domain, const SizeVec& vars, const std::vector<DiscreteFunction<Reward>>& maxQ);
  /// \brief Computes the (factored) max. Bellman Error via variable elimination (given an \a elimination_order over all state factors)
  double factoredBellmanError(const Domain& domain, const FactoredValueFunction& fval, const SizeVec& elimination_order);
}

/**
 * A planner that uses an approximate linear program (ALP) along with a factored value function.
 * Constraints are enumerated via variable elimination to avoid exponential LP size for large multi-agent problems.
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
  /// \brief The set of functions (\f$C\f$) appearing as nonlinear constraints in the LP
  std::vector<DiscreteFunction<Reward>> _C_set;
  /// \brief Discount factor
  float _gamma;
public:
  _ALPPlanner(const FactoredMDP& fmdp, float gamma)
  : _domain(fmdp->getDomain()), _fmdp(fmdp), _gamma(gamma) { }

  /// \brief Get best joint \a Action from joint \a State js
  virtual Action getAction(const State& js) override {
    assert(_value_fn != nullptr);
    return std::get<0>(_value_fn->getBestAction(js));
  }

  ///
  /// \brief run the FactoredALP algorithm: compute weights for \a FactoredValueFunction via approximate LP
  ///
  virtual int plan();
  /// \brief Compute backprojections and state relevance weights for LP
  void precompute();

  ///
  /// \brief Input a factored value function that will be used for solving
  ///
  void setFactoredValueFunction(FactoredValueFunction vfn) {
    _value_fn = std::move(vfn);
    _C_set.reserve(_value_fn->getBasis().size());
    _alpha.reserve(_C_set.size());
  }
  /// \brief Return value function associated with this ALP
  FactoredValueFunction getFactoredValueFunction() const {
    return _value_fn;
  }
  /// \brief Get the set of functions (\f$C\f$) from the ALP
  const std::vector<DiscreteFunction<Reward>>& getC() const {
    return _C_set;
  }
  /// \brief Get the set of local reward functions (\f$\mathbf{b}\f$) from the ALP
  const std::vector<DiscreteFunction<Reward>>& getLRFs() const {
    return _fmdp->getLRFs();
  }
  /// \brief Get the state relevance weights associated with the basis functions
  const std::vector<double> getAlpha() const {
    return _alpha;
  }
};
typedef boost::shared_ptr<_ALPPlanner> ALPPlanner;

/**
 * The Brute force (B) approximate linear program (ALP) planner along with a factored value function.
 * This planner is not smart about generating constraints with variable elimination but enumerates over all S, A
 */
class _BLPPlanner : public _ALPPlanner {
public:
  _BLPPlanner(const FactoredMDP& fmdp, float gamma)
  : _ALPPlanner(fmdp, gamma) { }

  ///
  /// \brief run the FactoredALP algorithm without optimized constraint generation.
  /// Compute weights for \a FactoredValueFunction via approximate LP after brute force constraint enumeration.
  ///
  virtual int plan() override;
};
#if 0
/**
 * The Vanilla (V) approximate linear program (ALP) planner along with a factored value function.
 * This planner is not smart about generating constraints with variable elimination but enumerates over all S, A
 * Further, it does not exploit local scopes in the basis functions (no efficient backprojections).
 * \note Only used for debugging purposes.
 */
class _VLPPlanner : public _ALPPlanner {
public:
  _VLPPlanner(const FactoredMDP& fmdp, float gamma)
  : _ALPPlanner(fmdp, gamma) { }

  ///
  /// \brief run the ALP algorithm without optimized constraint generation or backprojections.
  /// Compute weights for \a FactoredValueFunction via approximate LP after brute force constraint enumeration.
  ///
  virtual int plan() override;
};
#endif
/**
 * The SC-ALP (SCope-regularized Approximate integer Linear Program) planner along with a factored value function.
 * A generalization of the RALP in Petrik, Taylor, Parr, and Zilberstein, 2010, to support regularization of basis function scopes.
 * Constraints are enumerated via variable elimination to avoid exponential LP size for large multi-agent problems.
 */
class _SCALPPlanner : public _ALPPlanner {
protected:
  /// The L_1 regularization factor
  double _lambda;
  /// The basis function scope regularization factor
  double _beta;
  /// An upper bound on the maximum return (not reward) possible in this domain
  double _retBound;
public:
  _SCALPPlanner(const FactoredMDP& fmdp, float gamma, double lambda = 0., double beta = 0.)
  : _ALPPlanner(fmdp, gamma), _lambda(lambda), _beta(beta) {
    // compute upper bound on return
    _retBound = _domain->getRewardRange().getMax()/(1-_gamma);
  }

  ///
  /// \brief Compute weights for \a FactoredValueFunction via the scope-regularized approximate LP
  ///
  virtual int plan() override;
};

} // namespace crl

#endif
