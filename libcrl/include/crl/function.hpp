/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#ifndef FUNCTION_HPP_
#define FUNCTION_HPP_

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
  Domain _domain;
  /// \brief The subdomain relevant for this function
  /// \note Consists of state and action factors from the global domain
  Domain _subdomain;
  /// \brief The state factors relevant for this function, indicated by their absolution position in the global domain
  SizeVec _state_dom;
  /// \brief The action factors relevant for this function, indicated by their absolute position in the global domain
  SizeVec _action_dom;
  /// \brief Unique name of this function
  std::string _name;
  /// \brief True iff \a computeSubdomain() has been called
  bool _computed;
public:
  /// \brief ctor
  _DiscreteFunction(const Domain& domain, std::string name = "")
  : _domain(domain), _name(name), _computed(false) { }
  /// \brief Initialize this function's scope with the union of scopes of the ones in \a funcs
  _DiscreteFunction(std::initializer_list<_DiscreteFunction<T>> funcs, std::string name = "")
  : _name(name), _computed(false) {
    assert(funcs.size() != 0);
    _domain = funcs.begin()->_domain; // same domain
    for(const auto& func : funcs) {
      join(func);
    }
  }

  /// \brief Return subdomain associated with this function
  /// \note Only available after call to \a computeSubdomain()
  Domain getSubdomain() const {
    assert(_computed);
    return _subdomain;
  }
  /// \brief Computes the subdomain associated with this function
  void computeSubdomain() {
    _subdomain = boost::make_shared<_Domain>();
    const RangeVec& state_ranges = _domain->getStateRanges();
    const RangeVec& action_ranges = _domain->getActionRanges();
    const StrVec& state_names = _domain->getStateNames();
    const StrVec& action_names = _domain->getActionNames();
    for (Size i=0; i<_state_dom.size(); i++) {
        Size j = _state_dom[i];
        this->_subdomain->addStateFactor(state_ranges[j].getMin(), state_ranges[j].getMax(), state_names[j]);
    }
    for (Size i=0; i<_action_dom.size(); i++) {
        Size j = _action_dom[i];
        this->_subdomain->addActionFactor(action_ranges[j].getMin(), action_ranges[j].getMax(), action_names[j]);
    }
    _computed = true;
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
      _computed = false;
    }
  }
  /// \brief Add action factor `i' to the scope of this function
  virtual void addActionFactor(Size i) {
    assert(i < _domain->getNumActionFactors());
    // insert preserving order
    SizeVec::iterator it = std::lower_bound(_action_dom.begin(), _action_dom.end(), i);
    if(it == _action_dom.end() || *it != i) {
      _action_dom.insert(it, i);
      _computed = false;
    }
  }
  /// \brief Erase state factor `i' from the scope of this function
  virtual void eraseStateFactor(Size i) {
    assert(i < _state_dom.size());
    SizeVec::iterator it = std::lower_bound(_state_dom.begin(), _state_dom.end(), i);
    if(it != _state_dom.end()) {
      _state_dom.erase(it);
      _computed = false;
    }
  }
  /// \brief Erase action factor `i' from the scope of this function
  virtual void eraseActionFactor(Size i) {
    assert(i < _action_dom.size());
    SizeVec::iterator it = std::lower_bound(_action_dom.begin(), _action_dom.end(), i);
    if(it != _action_dom.end()) {
      _action_dom.erase(it);
      _computed = false;
    }
  }

  /// \brief Join state and action factor scopes of this function with the supplied ones
  virtual void join(const SizeVec& state_dom, const SizeVec& action_dom) {
    if(_state_dom != state_dom) {
        SizeVec joint_s;
        std::set_union(_state_dom.begin(), _state_dom.end(), state_dom.begin(), state_dom.end(), std::inserter(joint_s, joint_s.begin()));
        _state_dom = joint_s;
        _computed = false;
    }
    if(_action_dom != action_dom) {
        SizeVec joint_a;
        std::set_union(_action_dom.begin(), _action_dom.end(), action_dom.begin(), action_dom.end(), std::inserter(joint_a, joint_a.begin()));
        _action_dom = joint_a;
        _computed = false;
    }
  }
  /// \brief Join state and action factor scopes of this function with those of another one
  virtual void join(const _DiscreteFunction<T>& func) {
    join(func._state_dom, func._action_dom);
  }

  /// \brief True iff state factor i is included in the scope of this function
  bool containsStateFactor(Size i) const {
    return std::binary_search(_state_dom.begin(), _state_dom.end(), i); // sorted assumption
  }
  /// \brief True iff action factor i is included in the scope of this function
  bool containsActionFactor(Size i) const {
    return std::binary_search(_action_dom.begin(), _action_dom.end(), i); // sorted assumption
  }
  /// \brief The state factor indices (w.r.t. global \a Domain) relevant for this function
  const SizeVec& getStateFactors() const {
      return _state_dom;
  }
  /// \brief The action factor indices (w.r.t. global \a Domain) relevant for this function
  const SizeVec& getActionFactors() const {
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
    return eval(s, a);
  }
  /// \brief Convenience function for functions that do not depend on action variables
  virtual T operator()(const State& s) const {
    return eval(s, Action());
  }
};
// instead of typedef (which needs full type)
template<class T>
using DiscreteFunction = boost::shared_ptr<_DiscreteFunction<T>>;

template<class T>
using FunctionSetIterator = cpputil::MapValueRangeIterator<std::multimap<Size, DiscreteFunction<T>>, DiscreteFunction<T>>;

/**
 * \brief A many-to-many mapping from function scopes to particular functions
 * Used to retrieve functions with particular variables during variable elimination.
 * \todo compare with performance of unordered_multimap
 * FIXME check whether shared_ptr is needed to maintain elements in set (vs. raw pointers..)
 */
template<class T>
class FunctionSet : private std::multimap<Size, DiscreteFunction<T>> {
private:
  /// \brief type for iterator over multimap range
  typedef std::pair<typename std::multimap<Size,DiscreteFunction<T>>::iterator,
                    typename std::multimap<Size,DiscreteFunction<T>>::iterator> multimap_range;
  /// The domain which includes all state and action factors
  const Domain _domain;
  /// The index of the last state factor (where actions are inserted from)
  const Size _a_offset;
public:
  /// \brief type for iterator over functions in this \a FunctionSet
  typedef FunctionSetIterator<T> range;
  /// \brief ctor
  FunctionSet(const Domain& domain)
    : _domain(domain), _a_offset(_domain->getNumStateFactors()) { }

  /// \brief Add a function to this \a FunctionSet
  void insert(DiscreteFunction<T> f) {
    const SizeVec& s_scope = f->getStateFactors();
    const SizeVec& a_scope(f->getActionFactors());
    for(Size i : s_scope) {
        this->emplace(i,f);
    }
    for(Size i : a_scope) {
        this->emplace(i+_a_offset,f);
    }
  }

  /// \brief Erase all functions that depend on state factor `i'
  void eraseStateFactor(Size i) {
    this->erase(i);
  }
  /// \brief Erase all functions that depend on action factor `i'
  void eraseActionFactor(Size i) {
    this->erase(i+_a_offset);
  }
  /// \brief Get all functions that depend on state factor `i'
  range getStateFactor(Size i) {
    return range(this->equal_range(i));
  }
  /// \brief Get all functions that depend on action factor `i'
  range getActionFactor(Size i) {
    return range(this->equal_range(i+_a_offset));
  }

  /// \brief Number of elements in this std::multiset
  /// \note This is not equivalent to the number of functions contained in this set
  /// \see std::multiset<T>::size()
  typename std::multimap<Size, DiscreteFunction<T>>::size_type size() const {
    return std::multimap<Size, DiscreteFunction<T>>::size();
  }
  /// \brief Compute unique (both state and action) factors contained in this set
  typename std::multimap<Size, DiscreteFunction<T>>::size_type getNumFactors() const {
    typename std::multimap<Size, DiscreteFunction<T>>::size_type count = 0;
    for(auto it = this->begin(), end = this->end(); it != end; it = this->upper_bound(it->first)) {
        count++;
    }
    return count;
  }
  /// \brief Remove all elements from this std::multiset
  void clear() {
    return std::multimap<Size, DiscreteFunction<T>>::clear();
  }

};

/**
 * \brief This function is merely used for scope computations (e.g., variable elimination)
 */
template<class T>
class _EmptyFunction : public _DiscreteFunction<T> {
public:
  /// \brief ctor
  _EmptyFunction(const Domain& domain, std::string name = "")
  : _DiscreteFunction<T>(domain, name) { }
  _EmptyFunction(std::initializer_list<_DiscreteFunction<T>> funcs, std::string name = "")
  : _DiscreteFunction<T>(std::move(funcs), name) { }

  virtual T eval(const State& s, const Action& a) const override {
      throw cpputil::InvalidException("calling eval on an empty function");
  }
};
template<class T>
using EmptyFunction = boost::shared_ptr<_EmptyFunction<T>>;

//template<class T>
//using StateActionTable = boost::shared_ptr<_StateActionTable<T>>;

/**
 * \brief A \a DiscreteFunction implemented with tabular storage.
 * \todo Move into different header (function.hpp/common.hpp) and have DBNFactor/LRF inherit from this too
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

    ///
    /// \brief Assemble the flat table corresponding to this function
    /// \note Called after all state and action dependencies have been added
    ///
    virtual void pack() {
      if(!_DiscreteFunction<T>::_computed) {
          _DiscreteFunction<T>::computeSubdomain();
      }
      // allocate memory
      _sa_table = boost::make_shared<_FStateActionTable<T>>(this->_subdomain);
      _packed = true;
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

    virtual void addStateFactor(Size i) override {
        _DiscreteFunction<T>::addStateFactor(i);
        _packed = _DiscreteFunction<T>::_computed;
    }
    virtual void addActionFactor(Size i) override {
        _DiscreteFunction<T>::addActionFactor(i);
        _packed = _DiscreteFunction<T>::_computed;
    }
    virtual void eraseStateFactor(Size i) override {
        _DiscreteFunction<T>::eraseStateFactor(i);
        _packed = _DiscreteFunction<T>::_computed;
    }
    virtual void eraseActionFactor(Size i) override {
        _DiscreteFunction<T>::eraseActionFactor(i);
        _packed = _DiscreteFunction<T>::_computed;
    }
    virtual void join(const SizeVec& state_dom, const SizeVec& action_dom) override {
      _DiscreteFunction<T>::join(state_dom, action_dom);
      _packed = _DiscreteFunction<T>::_computed;
    }
    virtual void join(const _DiscreteFunction<T>& func) override {
      _DiscreteFunction<T>::join(func);
      _packed = _DiscreteFunction<T>::_computed;
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
  /// \brief Initialize an indicator on \a State s in a subdomain
  /// \param factors Denotes a (sub-)set of state factors from the supplied \a Domain
  _Indicator(const Domain& domain, SizeVec factors, const State& s, std::string name = "")
  : _DiscreteFunction(domain, name) {
    assert(s);
    std::sort(factors.begin(),factors.end());
    join(factors, this->getActionFactors());
    setState(s);
  }

  /// \brief Define the State for which this Indicator is set
  void setState(const State& s) {
    if(!_DiscreteFunction::_computed) {
      _DiscreteFunction::computeSubdomain();
    }
    _s_index = s; // copy index only-no other comparison (for identical domain, etc.) done
  }
  /// \brief Get the state (index) for which this Indicator is set
  Size getStateIndex() const {
      return _s_index;
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
      throw cpputil::InvalidException("Backprojection does not support function with action factors.");
    }
    // determine parent scope via DBN
    for(Size t : other->getStateFactors()) {
        const SizeVec& delayed_dep = _dbn->factor(t)->getDelayedDependencies();
        const SizeVec& action_dep = _dbn->factor(t)->getActionDependencies();
        _FDiscreteFunction<T>::join(delayed_dep, action_dep);
    }
  }

  /// \brief Compute backprojection for every variable setting in the domain
  void cache() {
      if(!_FDiscreteFunction<T>::_packed) {
        _FDiscreteFunction<T>::pack(); // allocate memory
      }
      // compute basis function values over its entire domain
      Domain hdom = _func->getSubdomain();
      _StateIncrementIterator hitr(hdom);
      std::vector<T> h(hdom->getNumStates(), 0); // the basis function cache over its domain
      // specialization for indicator functions
      Indicator I = boost::dynamic_pointer_cast<_Indicator>(_func);
      if(I) {
          h[I->getStateIndex()] = 1.;
          // TODO simplify the entire computation below as well for this special case
          // FIXME maintain special functions for sparse domains (ala these StateSetIterators !)
      }
      else {
        const Action empty_a;
        while (hitr.hasNext()) {
            const State& s = hitr.next();
            h[(Size)s] = _func->eval(s, empty_a);
        }
      }

      // compute backprojection, i.e., expectation of basis function through DBN
      _StateActionIncrementIterator saitr(this->_subdomain);
      const subdom_map h_dom(_func->getStateFactors());
      const subdom_map s_dom(this->getStateFactors());
      const subdom_map a_dom(this->getActionFactors());
      // efficient loop over all (s,a) pairs in backprojection domain
      auto& vals = this->_sa_table->values();
      for(T& v : vals) {
          const std::tuple<State,Action>& sa = saitr.next();
          v = 0.;
          hitr.reset();
          while(hitr.hasNext()) {
              const State& s = hitr.next();
              v += h[(Size)s] * _dbn->T(std::get<0>(sa), std::get<1>(sa), s, s_dom, h_dom, a_dom);
          }
      }
      _cached = true;
  }

  //
  // Function computations
  //

  virtual T eval(const State& s, const Action& a) const override {
    assert(_packed && _cached);
    return _FDiscreteFunction<T>::eval(s,a);
  }

  /// \brief Multiply all values with a scalar
  _Backprojection<T>& operator*=(T s) {
    assert(_packed && _cached);
    auto& vals = this->_sa_table->values();
    std::transform(vals.begin(), vals.end(), vals.begin(), [s](T v) { return v*s; });
    return *this;
  }
#if 0
  _Backprojection<T>& operator+=(const _FDiscreteFunction<T>& other) {
    assert(_packed && _cached);
    if(_DiscreteFunction<T>::getActionFactors() != other.getActionFactors() || DiscreteFunction<T>::_state_dom != other.getStateFactors()) {
        throw cpputil::InvalidException("");
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
