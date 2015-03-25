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
 * \brief The abstract interface for a basis function defined over a subset of state variables.
 */
template<class T>
class Basis {
protected:
  /// \brief The (global) domain which includes all state and action factors
  /// \note This is not necessarily equivalent to the (local) basis function scope
  const Domain _domain;
  /// \brief The (subset of) state factors relevant for this basis function
  SizeVec _state_dep;
public:
  /// \brief ctor
  Basis(const Domain& domain)
  : _domain(domain) { }
  /// \brief dtor
  virtual ~Basis() { }

  /// \brief Define the state factor scope for this basis function
  void addStateFactor(Size i) {
    assert(i < _domain->getNumStateFactors());
    _state_dep.push_back(i);
  }

  ///
  /// \brief evaluate the basis function at \a State s
  /// \note The number of state factor values in s must match the scope of this basis function.
  ///
  virtual T eval(const State& s) const = 0;
  /// \brief convenience function for evaluating this basis function
  virtual T operator()(const State& s) const {
    return eval(std::move(s));
  }
};

/**
 * \brief Indicator basis centered on a specific state
 */
class Indicator : public Basis<int> {
protected:
  /// \brief The state on which this indicator function is centered
  State _s;
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

} // namespace crl

#endif
