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
  const Domain _domain;
  /// \brief The subdomain relevant for this function
  /// \note Consists of state and action factors from the global domain
  Domain _subdomain;
  /// \brief The state factors relevant for this function, indicated by their absolution position in the global domain
  SizeVec _state_dom;
  /// \brief The action factors relevant for this function, indicated by their absolute position in the global domain
  SizeVec _action_dom;
  /// \brief Unique name of this function
  std::string _name;
public:
  /// \brief ctor
  _DiscreteFunction(const Domain& domain, std::string name = "")
  : _domain(domain), _name(name) { }
#if 0
  /// \brief Construct function from
  _DiscreteFunction(std::initializer_list<_DiscreteFunction<T>> funcs, std::string name = "")
  : _name(name) {
    for(const auto& func : funcs) {
      join(func);
    }
  }
#endif
  /// \brief dtor
  virtual ~_DiscreteFunction() { }

  /// \brief Return subdomain associated with this function
  virtual Domain getSubdomain() const {
    return _subdomain;
  }

  //
  // Implementation of the interface that maintains the scope (state and action factors) of this function
  //

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
  /// \brief Erase state factor `i' from the scope of this function
  virtual void eraseStateFactor(Size i) {
    assert(i < _state_dom.size());
    SizeVec::iterator it = std::lower_bound(_state_dom.begin(), _state_dom.end(), i);
    if(it != _state_dom.end()) {
      _state_dom.erase(it);
    }
  }
  /// \brief Erase action factor `i' from the scope of this function
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

  /// \brief True iff state factor i is included in the scope of this function
  virtual bool containsStateFactor(Size i) const {
    return std::binary_search(_state_dom.begin(), _state_dom.end(), i); // sorted assumption
  }
  /// \brief True iff action factor i is included in the scope of this function
  virtual bool containsActionFactor(Size i) const {
    return std::binary_search(_action_dom.begin(), _action_dom.end(), i); // sorted assumption
  }
  /// \brief The state factor indices (w.r.t. global \a Domain) relevant for this function
  virtual const SizeVec& getStateFactors() const {
      return _state_dom;
  }
  /// \brief The action factor indices (w.r.t. global \a Domain) relevant for this function
  virtual const SizeVec& getActionFactors() const {
      return _action_dom;
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
};
// instead of typedef (which needs full type)
template<class T>
using DiscreteFunction = boost::shared_ptr<_DiscreteFunction<T>>;

template<class T>
using StateActionTable =  boost::shared_ptr<_StateActionTable<T>>;

/**
 * \brief A \a DiscreteFunction implemented with tabular storage.
 * \todo Move into different header (function.hpp) and have DBNFactor/LRF inherit from this
 */
template<class T>
class _FDiscreteFunction : public _DiscreteFunction<T> {
protected:
    /// \brief The internal storage for this function
    boost::shared_ptr<_FStateActionTable<T>> _sa_table;
    /// \brief True iff \a pack() has been called (required for some function calls)
    bool _packed;
public:
    /// \brief ctor
    _FDiscreteFunction(const Domain& domain, std::string name = "")
    : _DiscreteFunction<T>(domain, name), _packed(false) { }
    /// \brief dtor
    virtual ~_FDiscreteFunction() { }

    ///
    /// \brief Assemble the flat table corresponding to this function
    /// \note Called after all state and action dependencies have been added
    ///
    virtual void pack() {
        this->_subdomain = boost::make_shared<_Domain>();
        const RangeVec& state_ranges = _DiscreteFunction<T>::_domain->getStateRanges();
        const RangeVec& action_ranges = _DiscreteFunction<T>::_domain->getActionRanges();
        const StrVec& state_names = _DiscreteFunction<T>::_domain->getStateNames();
        const StrVec& action_names = _DiscreteFunction<T>::_domain->getActionNames();
        for (Size i=0; i<_DiscreteFunction<T>::_state_dom.size(); i++) {
            Size j = _DiscreteFunction<T>::_state_dom[i];
            this->_subdomain->addStateFactor(state_ranges[j].getMin(), state_ranges[j].getMax(), state_names[j]);
        }
        for (Size i=0; i<_DiscreteFunction<T>::_action_dom.size(); i++) {
            Size j = _DiscreteFunction<T>::_action_dom[i];
            this->_subdomain->addActionFactor(action_ranges[j].getMin(), action_ranges[j].getMax(), action_names[j]);
        }
        // allocate memory
        _sa_table = boost::make_shared<_FStateActionTable<T>>(this->_subdomain);
        _packed = true;
    }

    /// \brief Return subdomain associated with this function
    /// \note Only available after call to \a pack()
    virtual Domain getSubdomain() const override {
      assert(_packed);
      return _DiscreteFunction<T>::getSubdomain();
    }

    ///
    /// \brief Define the function (s,a)->val<T>
    /// \note The number of state and action factor values in s must match the scope of this function
    ///
    void define(const State& s, const Action& a, const T& val) {
      assert(_packed);
      _sa_table->setValue(s,a, val);
    }
    virtual T eval(const State& s, const Action& a) const override {
      assert(_packed);
      return _sa_table->getValue(s,a);
    }

    //
    // invalidate tabular storage when function scope changes occur
    //

    virtual void addStateFactor(Size i) {
        _DiscreteFunction<T>::addStateFactor(i);
        _packed = false;
    }
    virtual void addActionFactor(Size i) {
        _DiscreteFunction<T>::addActionFactor(i);
        _packed = false;
    }
    virtual void eraseStateFactor(Size i) {
        _DiscreteFunction<T>::eraseStateFactor(i);
        _packed = false;
    }
    virtual void eraseActionFactor(Size i) {
        _DiscreteFunction<T>::eraseActionFactor(i);
        _packed = false;
    }
    virtual void join(const SizeVec& state_dom, const SizeVec& action_dom) override {
      _DiscreteFunction<T>::join(state_dom, action_dom);
      _packed = false;
    }
    virtual void join(const _DiscreteFunction<T>& func) override {
      _DiscreteFunction<T>::join(func);
      _packed = false;
    }
};

/**
 * \brief Indicator basis centered on a specific state
 */
class _Indicator : public _DiscreteFunction<double> {
protected:
  /// \brief The state on which this indicator function is centered
  Size _s_index;
public:
  /// \brief ctor
  _Indicator(const Domain& domain, std::string name = "")
  : _DiscreteFunction(domain, name) { }
  /// \brief Initialize an indicator on \a State s from the subdomain
  _Indicator(const Domain& domain, const Domain& subdomain, const State& s, std::string name = "")
  : _DiscreteFunction(domain, name) {
    assert(s);
    this->_subdomain = subdomain;
    _s_index = s;
  }
  virtual ~_Indicator() { }

  /// \brief Define the State for which this Indicator is set
  void set(const State& s) {
    assert(s);
    _subdomain = boost::make_shared<_Domain>();
    const RangeVec& state_ranges = this->_domain->getStateRanges();
    const StrVec& state_names = this->_domain->getStateNames();
    for (Size i=0; i<this->_state_dom.size(); i++) {
        Size j = this->_state_dom[i];
        _subdomain->addStateFactor(state_ranges[j].getMin(), state_ranges[j].getMax(), state_names[j]);
    }
    _s_index = s; // copy index only-no other comparison (for identical domain, etc.) done
  }

  virtual double eval(const State& s, const Action& a) const override {
    return static_cast<double>(_s_index == (Size)s); // note: only compares indices
  }
};
typedef boost::shared_ptr<_Indicator> Indicator;

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

/**
 * \brief Backprojection of a function through a DBN
 * The back-projection (the output) is defined over (state,action) or state factors alone.
 * \note Action factors in the input function are ignored.
 * \note DBNs with concurrent dependencies are currently not supported (should be minor change, though)
 */
template<class T>
class _Backprojection : public _FDiscreteFunction<T> {
protected:
  /// \brief The \a DBN used for backprojections
  DBN _dbn;
  /// \brief The \a DiscreteFunction to backproject
  DiscreteFunction<T> _func;
  /// \brief True iff function has been computed over entire domain.
  bool _cached;
public:
  _Backprojection(const Domain& domain, const DBN& dbn, const DiscreteFunction<T>& other, std::string name = "")
  : _FDiscreteFunction<T>(domain, name), _dbn(dbn), _func(other), _cached(false) {
    if(_dbn->hasConcurrentDependency()) {
      throw cpputil::InvalidException("Backprojection does currently not support concurrent dependencies in DBN.");
    }
    if(!other->getActionFactors().empty()) {
      std::cout << "[DEBUG]: action factors will be ignored during backprojection." << std::endl;
    }
    // determine parent scope via DBN
    for(Size t : other->getStateFactors()) {
        const SizeVec& delayed_dep = _dbn->factor(t)->getDelayedDependencies();
        const SizeVec& action_dep = _dbn->factor(t)->getActionDependencies();
        _FDiscreteFunction<T>::join(delayed_dep, action_dep);
    }
  }
  virtual ~_Backprojection() { }

  /// \brief Compute backprojection for every variable setting in the domain
  virtual void cache() {
      if(!this->_packed) {
        _FDiscreteFunction<T>::pack();
      }

      // compute basis function values over its entire domain

      // TODO: compute values
      //_StateIncrementIterator sitr(_parents);
      //_ActionIncrementIterator aitr(_parents);
      //while(sitr->)

      _cached = true;
  }

  //
  // Function computations
  //

  virtual T eval(const State& s, const Action& a) const override {
    assert(_cached);
    return _FDiscreteFunction<T>::eval(s,a);
  }

  /// \brief Multiply all values with a scalar
  virtual _Backprojection<T>& operator*=(T s) {
    assert(_cached);
    auto vals = this->_sa_table->values();
    std::transform(vals.begin(), vals.end(), vals.begin(), [s](T v) { return v*s; });
    return *this;
  }
#if 0
  virtual _Backprojection<T>& operator+=(const _DiscreteFunction<T>& other) {
    assert(_cached);
    if(_DiscreteFunction<T>::getActionFactors() != other.getActionFactors() || DiscreteFunction<T>::_state_dom != other.getStateFactors()) {
        throw cpputil::InvalidException("Backprojection does currently not support concurrent dependencies in DBN.");
    }
    return *this;
  }
#endif
};
// instead of typedef (which needs full type)
template<class T>
using Backprojection = boost::shared_ptr<_Backprojection<T>>;

} // namespace crl

#endif