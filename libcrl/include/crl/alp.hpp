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
 * \note Unlike (e.g.) the \a LRF or \a DBNFactor functions in dbn.hpp, this class supports modification of the scopes, e.g., for marginalization of variables.
 * \note Variables are sorted internally in ascending order
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
  /// \brief Unique name of this function
  std::string _name;
public:
  /// \brief ctor
  _DiscreteFunction(const Domain& domain, std::string name = "")
  : _domain(domain), _name(name) { }
  /// \brief Construct function from
  _DiscreteFunction(std::initializer_list<_DiscreteFunction<T>> funcs, std::string name = "")
  : _name(name) {
    for(const auto& func : funcs) {
      join(func);
    }
  }
  /// \brief dtor
  virtual ~_DiscreteFunction() { }

  /// \brief The state factor indices (w.r.t. global \a Domain) relevant for this function
  virtual const SizeVec& getStateFactors() const { return _state_dom; }
  /// \brief The action factor indices (w.r.t. global \a Domain) relevant for this function
  virtual const SizeVec& getActionFactors() const { return _action_dom; }

  //
  // Implementation of the interface that maintains the domain (state and action factors) of this function
  //

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

  /// \brief Join state and action factor scopes of this function with the supplied ones
  virtual void join(const SizeVec& state_dom, const SizeVec& action_dom) {
    if(_state_dom != state_dom) {
        SizeVec joint_s;
        std::set_union(_state_dom.begin(), _state_dom.end(), state_dom.begin(), state_dom.end(), joint_s.begin());
        _state_dom = joint_s;
    }
    if(_action_dom != action_dom) {
        SizeVec joint_a;
        std::set_union(_action_dom.begin(), _action_dom.end(), action_dom.begin(), action_dom.end(), joint_a.begin());
        _action_dom = joint_a;
    }
  }
  /// \brief Join state and action factor scopes of this function with those of another one
  virtual void join(const _DiscreteFunction<T>& func) {
    join(func._state_dom, func._action_dom);
  }

  /// \brief True iff state factor i is included in the domain of this function
  virtual bool containsStateFactor(Size i) const {
    return std::binary_search(_state_dom.begin(), _state_dom.end(), i); // sorted assumption
  }
  /// \brief True iff action factor i is included in the domain of this function
  virtual bool containsActionFactor(Size i) const {
    return std::binary_search(_action_dom.begin(), _action_dom.end(), i); // sorted assumption
  }

  //
  // Abstract interface for actual function computations
  //

  ///
  /// \brief Evaluate the function at \a State s and \a Action a.
  /// \note The number of state and action factor values in s must match the scope of this function
  ///
  virtual T eval(const State& s, const Action& a) const = 0;
  /// \brief Convenience function for evaluating this basis function
  virtual T operator()(const State& s, const Action& a) const {
    return eval(std::move(s), std::move(a));
  }
  /// \brief Convenience function for functions that do not depend on action variables
  virtual T operator()(const State& s) const {
    return eval(std::move(s), Action());
  }
  /// \brief Multiplication with a scalar
  //virtual _DiscreteFunction<T> operator*(T s) const = 0;
  virtual _DiscreteFunction<T>& operator*=(T s) = 0;
  /// \brief Addition of a function with the same domain
  //virtual _DiscreteFunction<T> operator+(const _DiscreteFunction<T>& f) const = 0;
  virtual _DiscreteFunction<T>& operator+=(const _DiscreteFunction<T>& f) = 0;
};
// instead of typedef (which needs full type)
template<class T>
using DiscreteFunction = boost::shared_ptr<_DiscreteFunction<T>>;

/**
 * \brief Indicator basis centered on a specific state
 */
class _Indicator : public _DiscreteFunction<double> {
protected:
  /// \brief The state on which this indicator function is centered
  State _s; // FIXME: don't copy, only store index perhaps
public:
  _Indicator(const Domain& domain, const State& s)
  : _DiscreteFunction(domain) {
    assert(s && s.size() == _state_dom.size());
    _s = s;
  }
  virtual ~_Indicator() { }

  virtual double eval(const State& s, const Action& a) const override {
    return static_cast<double>(_s == s); // note: internally also compares the ranges
  }
  virtual _DiscreteFunction<double>& operator*=(double s) override {
    return *this;
  }
  virtual _DiscreteFunction<double>& operator+=(const _DiscreteFunction<double>& f) override {
    return *this;
  }
};
#if 0
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
using StateActionTable =  boost::shared_ptr<_StateActionTable<T>>;
template<class T>
using StateTable =  boost::shared_ptr<_StateTable<T>>;

/**
 * \brief Backprojection of a function through a DBN
 * The input function is defined over state factors only.
 * The back-projection (the output) is defined over (state,action) or state factors alone.
 * Internally implemented with tabular storage.
 * \note DBNs with concurrent dependencies are currently not supported (should be minor change, though)
 */
template<class T>
class _Backprojection : public _DiscreteFunction<T> {
protected:
  /// \brief The \a DBN used for backprojections
  const _DBN& _dbn;
  /// \brief The \a DiscreteFunction to backproject
  const _DiscreteFunction<T>& _func;
  /// \brief The parent scope relevant for this state factor (consisting of both state and action variables)
  /// \note Only available after a call to \a cache()
  Domain _parents;
  /// \brief The internal storage for this backprojection
  union {
    StateTable<T> _state_table;
    StateActionTable<T> _state_action_table;
  };
  /// \brief Obtain element from internal storage
  /// \note This is a common interface independent of the storage type (StateTable, StateActionTable)
  std::function<T(std::initializer_list<RLType>)> getValue;
  /// \brief True iff function has been computed over entire domain.
  bool _cached;
public:
  _Backprojection(const Domain& domain, const _DBN& dbn, const _DiscreteFunction<T>& other, std::string name = "")
  : _DiscreteFunction<T>(domain, name), _dbn(dbn), _func(other), _cached(false) {
    assert(other.getActionFactors().empty());
    if(_dbn.hasConcurrentDependency()) {
      throw cpputil::InvalidException("Backprojection does currently not support concurrent dependencies in DBN.");
    }
    // determine parent scope via DBN
    for(Size t : other.getStateFactors()) {
        const SizeVec& delayed_dep = _dbn.factor(t)->getDelayedDependencies();
        const SizeVec& action_dep = _dbn.factor(t)->getActionDependencies();
        join(delayed_dep, action_dep);
    }
  }
  virtual ~_Backprojection() { }

  /// \brief Compute backprojection for every variable setting in the domain
  virtual void cache() {
    // allocate memory
    _parents = boost::make_shared<_Domain>();
    const RangeVec& state_ranges = _DiscreteFunction<T>::_domain->getStateRanges();
    const RangeVec& action_ranges = _DiscreteFunction<T>::_domain->getActionRanges();
    const StrVec& state_names = _DiscreteFunction<T>::_domain->getStateNames();
    const StrVec& action_names = _DiscreteFunction<T>::_domain->getActionNames();
    for (Size i=0; i<_DiscreteFunction<T>::_state_dom.size(); i++) {
        Size j = _DiscreteFunction<T>::_state_dom[i];
        _parents->addStateFactor(state_ranges[j].getMin(), state_ranges[j].getMax(), state_names[j]);
    }
    for (Size i=0; i<_DiscreteFunction<T>::_action_dom.size(); i++) {
        Size j = _DiscreteFunction<T>::_action_dom[i];
        _parents->addActionFactor(action_ranges[j].getMin(), action_ranges[j].getMax(), action_names[j]);
    }
    // define accessor function to internal memory
    if(_parents->getNumActionFactors() == 0) { // flat table in case of no action dependencies
      _state_table = boost::make_shared<_FStateTable<T>>(_parents);
      getValue = [&](std::initializer_list<RLType> params) {
        State s = *params.begin();
        return _state_table->getValue(std::move(s));
      };
    }
    else { // two-dimensional table in case of action dependencies
      _state_action_table = boost::make_shared<_FStateActionTable<T>>(_parents);
      getValue = [&](std::initializer_list<RLType> params) {
        auto it = params.begin();
        State s = *it++; // increment after dereference
        Action a = *it;
        return _state_action_table->getValue(std::move(s), std::move(a));
      };
    }

    // TODO: compute values
    //_StateIncrementIterator sitr(_parents);
    //_ActionIncrementIterator aitr(_parents);
    //while(sitr->)

    _cached = true;
  }

  virtual void join(const SizeVec& state_dom, const SizeVec& action_dom) override {
    _DiscreteFunction<T>::join(state_dom, action_dom);
    _cached = false;
  }
  virtual void join(const _DiscreteFunction<T>& func) override {
    _DiscreteFunction<T>::join(func);
    _cached = false;
  }

  //
  // Function computations
  //

  virtual T eval(const State& s, const Action& a) const override {
    assert(_cached);
    return getValue({s,a});
  }

  virtual _DiscreteFunction<T>& operator*=(T s) override {
    assert(_cached);
    return *this;
  }

  virtual _DiscreteFunction<T>& operator+=(const _DiscreteFunction<T>& f) override {
    assert(_cached);
    return *this;
  }

};
// instead of typedef (which needs full type)
template<class T>
using Backprojection = boost::shared_ptr<_Backprojection<T>>;

} // namespace crl

#endif
