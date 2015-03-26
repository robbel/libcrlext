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
#include "crl/common.hpp"
#include "crl/dbn.hpp"

namespace crl {

/**
 * \brief An abstract interface for a discrete function defined over a subset of (either state or action) variables.
 * Maps tuples {x_1,...,x_N,a_1,...,a_K} -> val<T>, where X_1,...,X_N are state variables and A_1,...,A_K action variables
 * that have been added to this function
 * \note Variables are sorted internally in ascending order; first all state followed by all action variables
 */
template<class T>
class _DiscreteFunction {
protected:
  /// \brief The (global) domain which includes all state and action factors
  /// \note This is not necessarily equivalent to the (local) function scope
  const Domain _domain;
  /// \brief The (subset of) state factors relevant for this function
  SizeVec _state_dom;
  /// \brief The (subset of) action factors relevant for this function
  SizeVec _action_dom;
public:
  /// \brief ctor
  _DiscreteFunction(const Domain& domain)
  : _domain(domain) { }
  /// \brief dtor
  virtual ~_DiscreteFunction() { }

  /// \brief Add state factor `i' to the scope of this function.
  virtual void addStateFactor(Size i) {
    assert(i < _domain->getNumStateFactors());
    // insert preserving order
    SizeVec::iterator it = std::lower_bound(_state_dom.begin(), _state_dom.end(), i);
    if(it == _state_dom.end() || *it != i) {
      _state_dom.insert(it, i);
    }
  }
  /// \brief Add action factor `i' to the scope of this function
  virtual void addActionFactor(Size i) {
    assert(i < _domain->getNumActionFactors());
    // insert preserving order
    SizeVec::iterator it = std::lower_bound(_action_dom.begin(), _action_dom.end(), i);
    if(it == _action_dom.end() || *it != i) {
      _action_dom.insert(it, i);
    }
  }

  /// \brief True iff state factor i is included in the domain of this function
  virtual bool containsStateFactor(Size i) const {
    return std::binary_search(_state_dom.begin(), _state_dom.end(), i); // sorted assumption
  }
  /// \brief True iff action factor i is included in the domain of this function
  virtual bool containsActionFactor(Size i) const {
    // sorted assumption
    return std::binary_search(_action_dom.begin(), _action_dom.end(), i); // sorted assumption
  }

  ///
  /// \brief evaluate the function at \a State s and \a Action a.
  /// \note The number of state and action factor values in s must match the scope of this function
  ///
  virtual T eval(const State& s, const Action& a) const = 0;
  /// \brief convenience function for evaluating this basis function
  virtual T operator()(const State& s, const Action& a) const {
    return eval(std::move(s), std::move(a));
  }
  /// \brief convenience function for functions that do not depend on action variables
  virtual T operator()(const State& s) const {
    return eval(std::move(s), Action());
  }
};
// instead of typedef (which needs full type)
template<class T>
using DiscreteFunction = boost::shared_ptr<_DiscreteFunction<T>>;
#if 0
/**
 * \brief Indicator basis centered on a specific state
 */
class Indicator : public Basis<int> {
protected:
  /// \brief The state on which this indicator function is centered
  State _s; // FIXME: don't copy, only store index perhaps
public:
  Indicator(const Domain& domain, const State& s)
  : Basis(domain) {
    assert(s && s.size() == _state_dep.size());
    _s = s;
  }
  virtual ~Indicator() { }

  virtual int eval(const State& s) const override {
    return static_cast<int>(_s == s); // note: internally also compares the ranges
  }
};

/**
 * \brief A factored value function
 */
class _FactoredV {
protected:
  /// \brief The (global) domain which includes all state and action factors
  const Domain _domain;
public:
  _FactoredV() { }
  virtual ~_FactoredV() { }

  /// \todo
  void addBasisFunction(const Basis<Reward>& h);

  /// \brief Return value of (global) \a State s
  virtual Reward eval(const State& s) const;
};
typedef boost::shared_ptr<_FactoredV> FactoredV;

/**
 * \brief Backprojection of a basis function through a DBN
 * Implemented with tabular storage.
 */
template<class T>
class _Backprojection : public _DBNFactor {
private:
  // trick to change visibility to private
  using _DBNFactor::addDelayedDependency;
  using _DBNFactor::addConcurrentDependency;
  using _DBNFactor::addActionDependency;
protected:

public:
  _Backprojection() { }
  virtual ~_Backprojection() { }

  /// \brief Compute backprojection for every
  void cache();

};
// instead of typedef (which needs full type)
template<class T>
using Backprojection = boost::shared_ptr<_Backprojection<T>>;
#endif
} // namespace crl

#endif
