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
 * \note Unlike (e.g.) the \a LRF function in dbn.hpp, this class indicates support for modification of the domain, e.g., for marginalization of variables.
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

  /// \brief Add state factor `i' to the domain of this function.
  virtual void addStateFactor(Size i) {
    assert(i < _domain->getNumStateFactors());
    // insert preserving order
    SizeVec::iterator it = std::lower_bound(_state_dom.begin(), _state_dom.end(), i);
    if(it == _state_dom.end() || *it != i) {
      _state_dom.insert(it, i);
    }
  }
  /// \brief Add action factor `i' to the domain of this function
  virtual void addActionFactor(Size i) {
    assert(i < _domain->getNumActionFactors());
    // insert preserving order
    SizeVec::iterator it = std::lower_bound(_action_dom.begin(), _action_dom.end(), i);
    if(it == _action_dom.end() || *it != i) {
      _action_dom.insert(it, i);
    }
  }
  /// \brief Erase state factor `i' from the domain of this function
  virtual void eraseStateFactor(Size i) {
    assert(i < _state_dom.size());
    SizeVec::iterator it = std::lower_bound(_state_dom.begin(), _state_dom.end(), i);
    if(it != _state_dom.end()) {
      _state_dom.erase(it);
    }
  }
  /// \brief Erase action factor `i' from the domain of this function
  virtual void eraseActionFactor(Size i) {
    assert(i < _action_dom.size());
    SizeVec::iterator it = std::lower_bound(_action_dom.begin(), _action_dom.end(), i);
    if(it != _action_dom.end()) {
      _action_dom.erase(it);
    }
  }

  /// \brief True iff state factor i is included in the domain of this function
  virtual bool containsStateFactor(Size i) const {
    return std::binary_search(_state_dom.begin(), _state_dom.end(), i); // sorted assumption
  }
  /// \brief True iff action factor i is included in the domain of this function
  virtual bool containsActionFactor(Size i) const {
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
#endif

template<class T>
using FlatTable = boost::shared_ptr<_FlatTable<T>>;

/**
 * \brief Backprojection of a function through a DBN
 * \note Both input type (I) and output type (O) can be specified.
 * Internally implemented with tabular storage.
 */
template<class I, class O>
class _Backprojection : public _DiscreteFunction<O> {
protected:
  /// \brief The \a DBN used for backprojections
  DBN _dbn;
  /// \brief The \a DiscreteFunction to backproject
  DiscreteFunction<I> _func;
  /// \brief The internal tabular storage for this function
  FlatTable<O> _val;
  /// \brief True iff function has been computed over entire domain.
  bool _cached;
public:
  _Backprojection(const DBN& dbn, const DiscreteFunction<I>& func)
  : _dbn(dbn), _func(func), _cached(false) { }
  virtual ~_Backprojection() { }

  /// \brief Compute backprojection for every variable setting in the domain
  virtual void cache();

  ///
  /// \brief evaluate the function at \a State s and \a Action a.
  /// \note The number of state and action factor values in s must match the scope of this function
  ///
  virtual O eval(const State& s, const Action& a) const override {
    assert(_cached);
  }
};
// instead of typedef (which needs full type)
template<class I, class O>
using Backprojection = boost::shared_ptr<_Backprojection<I,O>>;

} // namespace crl

#endif
