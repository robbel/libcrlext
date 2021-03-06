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
#include <cassert>
#include "crl/flat_tables.hpp"
#include "crl/common.hpp"
#include "crl/lifted_ops.hpp"

namespace crl {

//
// Forward declarations, typedefs
//

template<class T> class _DiscreteFunction;
template<class T> class FunctionSet;
/// \brief The identity mapping
static auto identity_map = [](Size i) { return i; };
/// \brief Comparison object for lifted factors
static auto lifted_comp = [](const LiftedFactor& l, const LiftedFactor& o) -> bool { return *l < *o; };
/// \brief A mapping from global factor indices to those in a subdomain
/// \see mapState(), mapAction()
typedef cpputil::inverse_map<Size> subdom_map;
// instead of typedef (which needs full type)
template<class T>
using DiscreteFunction = boost::shared_ptr<_DiscreteFunction<T>>;

//
// Algorithm declarations
//
namespace algorithm {

template<class T> T sum_over_domain(const crl::_DiscreteFunction<T>* pf, bool known_flat);
template<class T> DiscreteFunction<T> instantiate(const _DiscreteFunction<T>* pf, const State& s, bool known_flat);
template<class T> std::vector<T> slice(const _DiscreteFunction<T>* pf, Size i, const State&s, const Action& a);
template<class T> DiscreteFunction<T> maximize(const _DiscreteFunction<T>* pf, Size i, bool known_flat);
template<class T> DiscreteFunction<T> join(cpputil::Iterator<DiscreteFunction<T>>& funcs);
template<class T> std::tuple<Action,T> argVariableElimination(FunctionSet<T>& F, const crl::SizeVec& elimination_order);

}

// forward declarations
template<class T> std::ostream& operator<<(std::ostream& os, const _DiscreteFunction<T>& f);
template<class T> std::ostream& operator<<(std::ostream &os, const FunctionSet<T>& fset);

//
// Function definitions
//

/**
 * \brief An abstract interface for a discrete function defined over a subset of (either state or action) variables.
 * Maps tuples {x_1,...,x_N,a_1,...,a_K} -> val<T>, where X_1,...,X_N are state variables and A_1,...,A_K action variables
 * that have been added to this function. Additionally, lifted operations (e.g., generalized counters over variables) are supported.
 * \note Variables are sorted internally in ascending order
 * \note All methods in this class expect (s,a) tuples to be in the correct domain, no automatic conversions are applied.
 */
template<class T>
class _DiscreteFunction {
  // friend declarations
  friend DiscreteFunction<T> algorithm::instantiate<T>(const _DiscreteFunction<T>* pf, const State& s, bool known_flat);
  friend DiscreteFunction<T> algorithm::maximize<T>(const _DiscreteFunction<T>* pf, Size i, bool known_flat);
  friend DiscreteFunction<T> algorithm::join<T>(cpputil::Iterator<DiscreteFunction<T>>& funcs);
  friend std::vector<T> algorithm::slice<T>(const _DiscreteFunction<T>* pf, Size i, const State&s, const Action& a);
  friend std::tuple<Action,T> algorithm::argVariableElimination<T>(FunctionSet<T>& F, const crl::SizeVec& elimination_order);
  // operator overloads
  friend std::ostream& operator<< <>(std::ostream& os, const _DiscreteFunction<T>& f);
protected:
  /// \brief The (global) domain which includes all state and action factors
  Domain _domain;
  /// \brief The subdomain relevant for this function
  /// \note Consists of state and action factors from the global domain
  Domain _subdomain;
  /// \brief The state factors relevant for this function, indicated by their absolute position in the global domain
  SizeVec _state_dom;
  /// \brief The action factors relevant for this function, indicated by their absolute position in the global domain
  SizeVec _action_dom;
  /// \brief The lifted factors relevant for this function, sorted by their std::size_t hash value
  LiftedVec _lifted_dom;
  /// \brief Unique name of this function
  std::string _name;
  /// \brief True iff \a computeSubdomain() has been called
  bool _computed;

  /// \brief Helper function to add either state or action factor to this function's subdomain
  void addFactor(Size i, SizeVec& vec) {
    // insert preserving order
    SizeVec::iterator it = std::lower_bound(vec.begin(), vec.end(), i);
    if(it == vec.end() || *it != i) {
      vec.insert(it, i);
      _computed = false;
    }
  }
  /// \brief Helper function to erase either state or action factor from this function's subdomain
  /// \return True iff `i' existed in vec
  bool eraseFactor(Size i, SizeVec& vec) {
    SizeVec::iterator it = std::lower_bound(vec.begin(), vec.end(), i);
    if(it != vec.end() && (*it) == i) {
      vec.erase(it);
      _computed = false;
      return true;
    }
    return false;
  }
public:
  /// \brief ctor
  _DiscreteFunction(const Domain& domain, std::string name = "")
  : _domain(domain), _name(name), _computed(false) { }
  /// \brief Initialize this function's scope with the union of scopes of the ones in \a funcs
  _DiscreteFunction(cpputil::Iterator<DiscreteFunction<T>>& funcs, std::string name = "")
  : _name(name), _computed(false) {
    assert(funcs.hasNext());
    _domain = funcs.next()->_domain; // same domain
    funcs.reset();
    while(funcs.hasNext()) {
        join(*funcs.next());
    }
  }
  /// \brief copy ctor
  _DiscreteFunction(const _DiscreteFunction& rhs)
  : _domain(rhs._domain), _state_dom(rhs._state_dom), _action_dom(rhs._action_dom),
    _name(rhs._name), _computed(rhs._computed) {
      // copy these variables explicitly
      if(rhs._subdomain) {
        _subdomain = boost::make_shared<_Domain>(*rhs._subdomain);
      }
      for(const auto& lf : rhs._lifted_dom) {
        _lifted_dom.emplace_back(boost::make_shared<_LiftedFactor>(*lf));
      }
  }
  /// \brief copy assignment
  _DiscreteFunction& operator=(const _DiscreteFunction& rhs) {
    _DiscreteFunction<T> tmp(rhs);
    *this = std::move(tmp);
    return *this;
  }
  /// \brief move ctor
  _DiscreteFunction(_DiscreteFunction&&) = default;
  /// \brief move assignment
  _DiscreteFunction& operator=(_DiscreteFunction&&) = default;
  /// \brief dtor
  virtual ~_DiscreteFunction() { }

  /// \brief Return subdomain associated with this function
  /// \note Only available after call to \a computeSubdomain()
  Domain getSubdomain() const {
    assert(_computed); // FIXME: can probably compute this on the fly if not yet done. Makes interface simpler.
    return _subdomain;
  }
  std::string getName() const {
    return _name;
  }
  /// \brief Computes the subdomain associated with this function
  virtual void computeSubdomain() {
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
    // add lifted operators
    for (Size i=0; i<_lifted_dom.size(); i++) {
        FactorRange range = _lifted_dom[i]->getRange();
        this->_subdomain->addStateFactor(range.getMin(), range.getMax(), "#" + std::to_string(_lifted_dom[i]->getHash()));
    }
    _computed = true;
  }

  //
  // Helper functions for mapping between scopes
  //

  ///
  /// \brief Extract the relevant state information for this function (i.e., those corresponding to this \a _subdomain)
  /// \param s The current (e.g., joint) state
  /// \param domain_map An (optional) function mapping state factor Ids in the global _domain to those in the supplied \a State s
  /// \note Only available after call to \a computeSubdomain()
  /// \note When the size of state s corresponds to size of local state space, s is directly returned (optimization)
  /// \note When the size of state s does not correspond to size of local state space, the provided \a domain_map is used for resolution.
  ///
  template <class C>
  State mapState(const State& s, C&& domain_map_s) const {
      assert(_computed);
      if(s.size() == _state_dom.size() + _lifted_dom.size()) // under this condition no reduction to local scope performed
          return s;
      State ms(_subdomain);
      for (Size i = 0; i<_state_dom.size(); i++) {
          Size j = _state_dom[i];
          ms.setFactor(i, s.getFactor(domain_map_s(j)));
      }
      // lifted operators
      for (Size i = 0; i<_lifted_dom.size(); i++) {
          auto hash = _lifted_dom[i]->getHash();
          ms.setFactor(i+_state_dom.size(), s.getFactor(domain_map_s(hash)));
      }
      return ms;
  }
  State mapState(const State& s) const {
      return mapState(s, identity_map);
  }

  ///
  /// \brief Extract the relevant action information for this function (i.e., those corresponding to this \a _subdomain)
  /// \param a The (e.g., joint) action
  /// \param domain_map An (optional) function mapping action factor Ids in the global _domain to those in the supplied \a Action a
  /// \note Only available after call to \a computeSubdomain()
  /// \note When the size of action a corresponds to size of local action space, a is directly returned (optimization)
  /// \note When the size of action a does not correspond to size of local state space, the provided \a domain_map is used for resolution.
  /// FIXME test/fix for empty action -- just returning `a' may be sufficient!
  ///
  template <class C>
  Action mapAction(const Action& a, C&& domain_map) const {
      assert(_computed);
      if(a.size() == _action_dom.size()) // under this condition no reduction to local scope performed
          return a;
      Action ma(_subdomain);
      for (Size i=0; i<_action_dom.size(); i++) {
          Size j = _action_dom[i];
          ma.setFactor(i, a.getFactor(domain_map(j)));
      }
      return ma;
  }
  Action mapAction(const Action& a) const {
      return mapAction(a, identity_map);
  }

  //
  // Implementation of the interface that maintains the scope (state and action factors) of this function
  //

  /// \brief Add state factor `i' to the scope of this function.
  virtual void addStateFactor(Size i) {
    assert(i < _domain->getNumStateFactors());
    addFactor(i, _state_dom);
  }
  /// \brief Add action factor `i' to the scope of this function
  virtual void addActionFactor(Size i) {
    assert(i < _domain->getNumActionFactors());
    addFactor(i, _action_dom);
  }
  /// \brief Add a lifted factor to the scope of this function
  virtual void addLiftedFactor(LiftedFactor lf) {
    // insert preserving order
    LiftedVec::iterator it = std::lower_bound(_lifted_dom.begin(), _lifted_dom.end(), lf, lifted_comp);
    if(it == _lifted_dom.end() || *(*it) != *lf) {
#if !NDEBUG
      // no support for hash collisions between regular state Ids and lifted operations
      // note: assumes regular states have been added in full
      SizeVec::iterator sit = std::lower_bound(_state_dom.begin(), _state_dom.end(), lf->getHash());
      assert(sit == _state_dom.end() || (*sit) != lf->getHash());
#endif
      _lifted_dom.emplace(it, lf);
      _computed = false;
    }
  }
  /// \brief Erase state factor `i' from the scope of this function
  virtual bool eraseStateFactor(Size i) {
    assert(i < _domain->getNumStateFactors());
    return eraseFactor(i, _state_dom);
  }
  /// \brief Erase action factor `i' from the scope of this function
  virtual bool eraseActionFactor(Size i) {
    assert(i < _domain->getNumActionFactors());
    return eraseFactor(i, _action_dom);
  }
  /// \brief Erase action or state factor `i' from the scope of this function
  /// \note If `i' is greater than the number of state factors in the (global) domain, it is assumed to be an action factor.
  bool eraseFactor(Size i) {
      if(i < _domain->getNumStateFactors()) {
          return eraseStateFactor(i);
      }
      else {
          return eraseActionFactor(i - _domain->getNumStateFactors());
      }
  }
  /// \brief Reduce the domain of every lifted factor that contains state factor `i'
  /// \note If a lifted factor is reduced to the empty domain, it is fully removed from this function
  /// \return An array of <old_hash,new_hash> (sorted by the former) for those lifted factors modified by this function
  virtual std::vector<std::pair<std::size_t,std::size_t>> eraseLiftedFactor(Size i) {
    assert(i < _domain->getNumStateFactors());
    std::vector<std::pair<std::size_t,std::size_t>> retVec;
    for(auto it = _lifted_dom.begin(); it != _lifted_dom.end(); ) { // sorted
      auto o_hash = (*it)->getHash(); // old hash
      if((*it)->eraseStateFactor(i)) {
        _computed = false;
      }
      auto n_hash = (*it)->getHash(); // new hash
      if((*it)->empty()) {
        it = _lifted_dom.erase(it);
        n_hash = _LiftedFactor::EMPTY_HASH;
      }
      else {
        ++it;
      }
      if(o_hash != n_hash) {
        retVec.emplace_back(std::make_pair(o_hash, n_hash));
      }
    }

    if(!retVec.empty()) {
      // resort lifted op vector if it was modified
      std::sort(_lifted_dom.begin(),_lifted_dom.end(),lifted_comp);
      // check whether duplicate counter scope has been created
      LiftedFactor lf = boost::make_shared<_LiftedFactor>(std::initializer_list<Size>{});
      for(const auto& p : retVec) {
        if(p.second == _LiftedFactor::EMPTY_HASH) {
            continue;
        }
        lf->setHash(p.second);
        auto r = std::equal_range(_lifted_dom.begin(),_lifted_dom.end(),lf,lifted_comp);
        if(std::distance(r.first,r.second)>1) { // duplicate counter
          _lifted_dom.erase(r.first);
        }
      }
    }
    return retVec;
  }
  /// \brief Erase lifted factor `lf' from the (lifted) scope of the function
  virtual void eraseLiftedFactor(const LiftedFactor& lf) {
    LiftedVec::iterator it = std::lower_bound(_lifted_dom.begin(), _lifted_dom.end(), lf, lifted_comp);
    if(it != _lifted_dom.end() && *(*it) == *lf) {
      _lifted_dom.erase(it);
      _computed = false;
    }
  }
  /// \brief Compress this function by selectively removing variables from lifted scopes and introducing them as proper variables.
  /// Greedy algorithm terminates when compression is maximal
  /// \param delVars If supplied, this `old_hash -> var' mapping is filled with the variables deleted from each counter during compression
  /// \return An array of <old_hash,new_hash> (sorted by the former) for those lifted factors modified by this function
  virtual std::vector<std::pair<std::size_t,std::size_t>> compress(std::unordered_multimap<std::size_t,Size>* delVars = nullptr) {
    static std::vector<bool> s_vars(_domain->getNumStateFactors());
    typedef std::unordered_multimap<Size,Size> SizeMap;
    // fill bit-vector of enabled proper variables
    std::fill(s_vars.begin(), s_vars.end(), 0);
    for(auto i : getStateFactors()) {
        s_vars[i] = true;
    }

    SizeVec counter_sz(_lifted_dom.size()); // counter scope sizes
    std::vector<std::size_t> o_hashes(_lifted_dom.size()); // counter hashes before mod
    SizeVec counter_sz_red(_lifted_dom.size()); // reduced counter scope sizes
    SizeMap uom; // state factor to (multiple) counter mapping
    SizeVec::size_type i = 0;
    for(auto it = _lifted_dom.begin(); it != _lifted_dom.end(); ++it) { // sorted
        o_hashes[i] = (*it)->getHash();
        counter_sz[i] = (*it)->getStateFactors().size()+1;
        counter_sz_red[i] = counter_sz[i]-1;
        assert(counter_sz_red[i] > 0);
        for(Size j : (*it)->getStateFactors()) {
          uom.emplace(j,i);
        }
        i++;
    }
    // determine next variable to promote to `proper' TODO: does this require next `best' choice?
    std::vector<std::size_t> n_hashes(o_hashes); // counter hashes after mod
    for(auto pit = uom.begin(); pit != uom.end(); ) {
        const Size var = pit->first; // var under consideration
        const auto range = uom.equal_range(var);
        const auto end = range.second;
        // size comparison
        Size sz = 1;
        Size sz_red = 2;
        for (; pit != end; ++pit) {
            sz *= counter_sz[pit->second];
            sz_red *= counter_sz_red[pit->second];
        }
        if(s_vars[var] || sz_red < sz) { // if analogous `proper' variable already exists or better size possible
            addStateFactor(var);
            // erase element from corresponding counters
            std::for_each(range.first, range.second, [&](SizeMap::value_type& v) {
              assert(_lifted_dom[v.second]->containsStateFactor(v.first));
              _lifted_dom[v.second]->eraseStateFactor(v.first);
              auto n_hash = _lifted_dom[v.second]->getHash(); // new hash
              if(_lifted_dom[v.second]->empty()) {
                  n_hash = _LiftedFactor::EMPTY_HASH;
              }
              n_hashes[v.second] = n_hash;
              // update counters
              counter_sz[v.second] = counter_sz_red[v.second];
              counter_sz_red[v.second] = counter_sz_red[v.second] > 1 ? counter_sz_red[v.second]-1 : 1;
              // optionally store deleted variables
              if(delVars) {
                delVars->emplace(o_hashes[v.second],v.first);
              }
            });
            // update mapping
            uom.erase(range.first, range.second);
            _computed = false;
            pit = uom.begin(); // restart passes over uom
        }
    }

    std::vector<std::pair<std::size_t,std::size_t>> retVec;
    for(int i = 0; i < o_hashes.size(); i++) {
        if(o_hashes[i] != n_hashes[i]) {
          retVec.emplace_back(std::make_pair(o_hashes[i], n_hashes[i]));
        }
    }

    // clean up modified _lifted_dom
    if(!retVec.empty()) {
      std::sort(_lifted_dom.begin(),_lifted_dom.end(),lifted_comp); // re-sort
      // check whether duplicate counter scope has been created
      LiftedFactor lf = boost::make_shared<_LiftedFactor>(std::initializer_list<Size>{});
      for(const auto& p : retVec) {
        lf->setHash(p.second);
        if(p.second == _LiftedFactor::EMPTY_HASH) {
            eraseLiftedFactor(lf);
        }
        else {
            auto r = std::equal_range(_lifted_dom.begin(), _lifted_dom.end(), lf, lifted_comp);
            if(std::distance(r.first,r.second)>1) { // duplicate counter
                _lifted_dom.erase(r.first,r.second-1); // leave only one in _lifted_dom
            }
        }
      }
    }
    return retVec;
  }

  /// \brief Join state and action factor scopes of this function with the supplied ones
  virtual void join(const SizeVec& state_dom, const SizeVec& action_dom, const LiftedVec& lifted_dom) {
    if(_state_dom != state_dom) {
        SizeVec joint_s;
        std::set_union(_state_dom.begin(), _state_dom.end(), state_dom.begin(), state_dom.end(), std::inserter(joint_s, joint_s.begin()));
        _state_dom = std::move(joint_s);
        _computed = false;
    }
    if(_action_dom != action_dom) {
        SizeVec joint_a;
        std::set_union(_action_dom.begin(), _action_dom.end(), action_dom.begin(), action_dom.end(), std::inserter(joint_a, joint_a.begin()));
        _action_dom = std::move(joint_a);
        _computed = false;
    }
    // lifted operations
    if(_lifted_dom.size() != lifted_dom.size() ||
       !std::equal(_lifted_dom.begin(), _lifted_dom.end(), lifted_dom.begin(), [](const LiftedFactor& a, const LiftedFactor& b) { return *a == *b; })) {
      LiftedVec joint_l;
      std::set_union(_lifted_dom.begin(), _lifted_dom.end(), lifted_dom.begin(), lifted_dom.end(), std::inserter(joint_l, joint_l.begin()), lifted_comp);
      // copy these variables explicitly
      _lifted_dom.clear();
      for(const auto& lf : joint_l) {
        _lifted_dom.emplace_back(boost::make_shared<_LiftedFactor>(*lf));
      }
      _computed = false;
    }
  }
  /// \brief Join state and action factor scopes of this function with those of another one
  virtual void join(const _DiscreteFunction<T>& func) {
    join(func._state_dom, func._action_dom, func._lifted_dom);
  }

  /// \brief True iff state factor i is included in the scope of this function
  bool containsStateFactor(Size i) const {
    return std::binary_search(_state_dom.begin(), _state_dom.end(), i); // sorted assumption
  }
  /// \brief True iff action factor i is included in the scope of this function
  bool containsActionFactor(Size i) const {
    return std::binary_search(_action_dom.begin(), _action_dom.end(), i); // sorted assumption
  }
  /// \brief True iff lifted factor i is included in the scope of this function
  bool containsLiftedFactor(const LiftedFactor& lf) const {
    return std::binary_search(_lifted_dom.begin(), _lifted_dom.end(), lf, lifted_comp); // sorted assumption
  }
  /// \brief The state factor indices (w.r.t. global \a Domain) relevant for this function
  const SizeVec& getStateFactors() const {
      return _state_dom;
  }
  /// \brief The action factor indices (w.r.t. global \a Domain) relevant for this function
  const SizeVec& getActionFactors() const {
      return _action_dom;
  }
  /// \brief The lifted factors relevant for this function
  const LiftedVec& getLiftedFactors() const {
      return _lifted_dom;
  }

  //
  // Abstract interface for actual function computations
  //

  ///
  /// \brief Evaluate the function at \a State s and \a Action a.
  /// \note The number of state and action factor values in s must match the scope of this function
  ///
  virtual T eval(const State& s, const Action& a) const = 0;
  /// \brief Convenience function for functions that do not depend on action variables
  virtual T eval(const State& s) const {
    return eval(s, Action());
  }
  /// \brief Convenience function for functions that do not depend on state variables
  virtual T eval(const Action& a) const {
    return eval(State(), a);
  }
  /// \brief Convenience function for evaluating this basis function
  virtual T operator()(const State& s, const Action& a) const {
    return eval(s, a);
  }
  /// \brief Convenience function for functions that do not depend on action variables
  virtual T operator()(const State& s) const {
    return eval(s, Action());
  }
  /// \brief Convenience function for functions that do not depend state variables
  virtual T operator()(const Action& a) const {
    return eval(State(), a);
  }
};

template<class T>
std::ostream& operator<<(std::ostream &os, const _DiscreteFunction<T>& f) {
  os << "f(S,A)=f({";
  for(auto s : f._state_dom) {
    os << s << ", ";
  }
  os << "},{";
  for(auto a : f._action_dom) {
    os << a << ", ";
  }
  os << "}";
  if(!f._lifted_dom.empty()) {
    os << ",";
    for(auto l : f._lifted_dom) {
      os << *l << ", ";
    }
  }
  os << ")";
  return os;
}

template<class T>
using FunctionSetIterator = cpputil::MapValueRangeIterator<DiscreteFunction<T>, std::multimap<Size, DiscreteFunction<T>>>;

/**
 * \brief A many-to-many mapping from function scopes to particular functions
 * Used to retrieve functions with particular variables during variable elimination.
 * \todo compare with performance of unordered_multimap
 */
template<class T>
class FunctionSet : private std::multimap<Size, DiscreteFunction<T>> {
  // operator overloads
  friend std::ostream& operator<< <>(std::ostream &os, const FunctionSet<T>& fset);
private:
  /// \brief Iterator type for this multimap
  typedef typename std::multimap<Size, DiscreteFunction<T>>::iterator fset_iterator;
  /// \brief The domain which includes all state and action factors
  const Domain _domain;
  /// \brief The index of the last state factor (where actions are inserted from)
  const Size _a_offset;
  /// \brief Reverse lookup from an inserted \a DiscreteFunction to its iterators in the multimap
  std::multimap<DiscreteFunction<T>, fset_iterator> reverse_lookup;
  /// \brief Iterator for reverse lookups
  typedef std::pair<typename std::multimap<DiscreteFunction<T>, fset_iterator>::iterator,
                    typename std::multimap<DiscreteFunction<T>, fset_iterator>::iterator> reverse_range;
  /// \brief Iterator for forward lookups
  typedef std::pair<typename std::multimap<Size, DiscreteFunction<T>>::iterator,
                    typename std::multimap<Size, DiscreteFunction<T>>::iterator> forward_range;
public:
  /// \brief type for iterator over functions in this \a FunctionSet
  using range = FunctionSetIterator<T>;
  /// \brief ctor
  FunctionSet(const Domain& domain)
    : _domain(domain), _a_offset(_domain->getNumStateFactors()) { }

  /// \brief Add a function to this \a FunctionSet
  void insert(DiscreteFunction<T> f) {
    const SizeVec& s_scope = f->getStateFactors();
    const SizeVec& a_scope = f->getActionFactors();
    const LiftedVec& l_vec = f->getLiftedFactors();
    for(Size i : s_scope) {
        auto it = this->emplace(i,f);
        reverse_lookup.emplace(f,it);
    }
    for(Size i : a_scope) {
        auto it = this->emplace(i+_a_offset,f);
        reverse_lookup.emplace(f,it);
    }
    // account for lifted operations
    SizeVec unique_lf;
    for(const auto& lf : l_vec) {
        const SizeVec& s_dom = lf->getStateFactors();
        unique_lf.insert(unique_lf.end(),s_dom.begin(),s_dom.end());
    }
    std::sort(unique_lf.begin(),unique_lf.end());
    auto last = std::unique(unique_lf.begin(), unique_lf.end());
    for(auto uit = unique_lf.begin(); uit != last; ++uit) {
        // enforce uniqueness w.r.t. `proper' variables
        if(!std::binary_search(s_scope.begin(), s_scope.end(), *uit)) {
          auto it = this->emplace(*uit,f);
          reverse_lookup.emplace(f,it);
        }
    }
  }

  /// \brief Erase all functions that depend on state factor `i'
  void eraseStateFactor(Size i) {
    assert(i < _a_offset);
    eraseFactor(i);
  }
  /// \brief Erase all functions that depend on action factor `i'
  void eraseActionFactor(Size i) {
    eraseFactor(i+_a_offset);
  }
  /// \brief Erase all functions that depend on factor `i'
  /// \note If `i' is greater than the number of state factors, it is assumed to be an action factor.
  void eraseFactor(Size i) {
    forward_range r = this->equal_range(i); // look up all such functions
    while(r.first!=r.second) {
        const DiscreteFunction<T>& f = r.first->second;
        const reverse_range rr = reverse_lookup.equal_range(f); // list of iterators to delete
        for(auto riter = rr.first; riter != rr.second; ) {
            bool erased = false;
            if(r.first == riter->second) { // invalidating myself
              r.first = this->erase(riter->second);
              erased = true;
            }
            if(r.second == riter->second) { // invalidating the end of the range
                if(!erased) {
                    r.second = this->erase(riter->second);
                }
                else {
                    r.second = r.first;
                }
                erased = true;
            }
            if(!erased) { // no iterator will be invalidated by this erase
              this->erase(riter->second);
            }
            ++riter;
        }
        // clean up reverse map (optional)
        reverse_lookup.erase(rr.first, rr.second);
    }
  }
  /// \brief Get all functions that depend on state factor `i'
  range getStateFactor(Size i) {
    assert(i < _a_offset);
    return range(this->equal_range(i));
  }
  /// \brief Get all functions that depend on action factor `i'
  range getActionFactor(Size i) {
    return range(this->equal_range(i+_a_offset));
  }
  /// \brief Get all functions that depend on factor `i'.
  /// \note If `i' is greater than the number of state factors, it is assumed to be an action factor.
  range getFactor(Size i) {
    return range(this->equal_range(i));
  }

  /// \brief Number of elements in this std::multiset
  /// \note This is not equivalent to the number of functions contained in this set
  /// \see std::multiset<T>::size()
  typename std::multimap<Size, DiscreteFunction<T>>::size_type size() const {
    return std::multimap<Size, DiscreteFunction<T>>::size();
  }
  /// \brief Compute unique (both state and action) factors contained in this set
  SizeVec getFactors() const {
    SizeVec vec;
    for(auto it = this->begin(), end = this->end(); it != end; it = this->upper_bound(it->first)) {
        vec.push_back(it->first);
    }
    return vec;
  }
  /// \brief Return a vector of all unique functions contained in this set
  std::vector<DiscreteFunction<T>> getFunctions() const {
    std::vector<DiscreteFunction<T>> vec;
    for(auto it = reverse_lookup.begin(), end = reverse_lookup.end(); it != end; it = reverse_lookup.upper_bound(it->first)) {
        vec.push_back(it->first);
    }
    return vec;
  }
  /// \brief Compute number of unique (both state and action) factors contained in this set
  typename std::multimap<Size, DiscreteFunction<T>>::size_type getNumFactors() const {
    return getFactors().size();
  }

  /// \brief Remove all elements from this std::multiset
  void clear() {
    std::multimap<Size, DiscreteFunction<T>>::clear();
    reverse_lookup.clear();
  }
  /// \brief True iff this \a FunctionSet is empty
  bool empty() const {
    return std::multimap<Size, DiscreteFunction<T>>::empty();
  }
};

template<class T>
std::ostream& operator<<(std::ostream &os, const FunctionSet<T>& fset) {
  os << "fset[ ";
  for(const auto& f : fset) {
      os << "[ " << f.first << ": " << *f.second << " ]";
  }
  os << " ]";
  return os;
}

/**
 * \brief This function is merely used for scope computations (e.g., variable elimination)
 */
template<class T>
class _EmptyFunction : public _DiscreteFunction<T> {
public:
  /// \brief ctor
  _EmptyFunction(const Domain& domain, std::string name = "")
  : _DiscreteFunction<T>(domain, name) { }
  _EmptyFunction(cpputil::Iterator<DiscreteFunction<T>>& funcs, std::string name = "")
  : _DiscreteFunction<T>(funcs, name) { }

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
 */
template<class T>
class _FDiscreteFunction : public _DiscreteFunction<T> {
    friend T algorithm::sum_over_domain<T>(const _DiscreteFunction<T>* pf, bool known_flat);
    friend DiscreteFunction<T> algorithm::instantiate<T>(const _DiscreteFunction<T>* pf, const State& s, bool known_flat);
protected:
    /// \brief The internal storage for this function
    boost::shared_ptr<_FStateActionTable<T>> _sa_table;
    /// \brief True iff \a pack() has been called (required for some function calls)
    bool _packed;
    /// \brief General in-place transformation method with another function
    /// \param known_flat True iff `other' is known to be a _FDiscreteFunction and special optimizations apply
    /// \note Supports the case that function \a other is defined over a (proper) subset of factors from this function
    template<class F>
    void transform(const _DiscreteFunction<T>* other, F f, bool known_flat) {
      auto& vals = _sa_table->values();
      // optional rtti check to allow optimizations below
      const _FDiscreteFunction<T>* fother;
      if(known_flat) {
          fother = static_cast<const _FDiscreteFunction<T>*>(other);
      } else {
          fother = dynamic_cast<const _FDiscreteFunction<T>*>(other);
          if(fother) {
              known_flat = true;
          }
      }
      // implementation
      if(known_flat && vals.size() == fother->_sa_table->values().size()) { // efficient transform possible
          std::transform(vals.begin(), vals.end(), fother->_sa_table->values().begin(), vals.begin(), f);
          return;
      }

      // apply a function with smaller domain TODO optimize for `other function is flat' case
      if(std::includes(this->getStateFactors().begin(), this->getStateFactors().end(), // check if other domain is proper subset
                       other->getStateFactors().begin(), other->getStateFactors().end()) &&
         std::includes(this->getLiftedFactors().begin(), this->getLiftedFactors().end(),
                       other->getLiftedFactors().begin(), other->getLiftedFactors().end(), lifted_comp)) {
          const Size num_actions = this->_subdomain->getNumActions();
          subdom_map s_dom(this->getStateFactors()); // assumed to subsume all states from function `other'
          for(const auto& lf : this->getLiftedFactors()) { // assumed to subsume all lifted factors from function `other'
              s_dom.append(lf->getHash());
          }
          if(other->getActionFactors().empty()) {
              _StateIncrementIterator sitr(this->_subdomain);
              while(sitr.hasNext()) {
                  const State& s = sitr.next();
                  State ms = other->mapState(s, s_dom);
                  T val = other->eval(ms); // evaluate `other' function
                  T& start = vals[s.getIndex()*num_actions]; // fill from here
                  auto start_it = vals.begin() + (&start - vals.data());
                  std::transform(start_it, start_it+num_actions, start_it, std::bind2nd(f, val));
              }
          }
          else if(std::includes(this->getActionFactors().begin(), this->getActionFactors().end(), // check if other domain is proper subset
                                other->getActionFactors().begin(), other->getActionFactors().end())) {
              subdom_map a_dom(this->getActionFactors()); // assumed to subsume all states from function `other'
              _StateActionIncrementIterator saitr(this->_subdomain);
              while(saitr.hasNext()) {
                  const std::tuple<State,Action>& sa = saitr.next();
                  const State& s = std::get<0>(sa);
                  const Action& a = std::get<1>(sa);
                  State ms = other->mapState(s, s_dom);
                  Action ma = other->mapAction(a, a_dom);
                  T val = other->eval(ms,ma); // evaluate `other' function
                  const auto idx = s.getIndex()*num_actions+a.getIndex();
                  vals[idx] = f(vals[idx],val);
              }
          }
      }
      else {
          throw cpputil::InvalidException("Operation on two given functions not currently implemented.");
      }
    }
public:
    /// \brief ctor
    _FDiscreteFunction(const Domain& domain, std::string name = "")
    : _DiscreteFunction<T>(domain, name), _packed(false) { }
    /// \brief copy ctor
    _FDiscreteFunction(const _FDiscreteFunction<T>& rhs)
    : _DiscreteFunction<T>(rhs), _packed(rhs._packed) {
        if(rhs._sa_table) {
            _sa_table = boost::make_shared<_FStateActionTable<T>>(*rhs._sa_table); // deep copy
        }
    }
    /// \brief copy assignment
    _FDiscreteFunction& operator=(const _FDiscreteFunction& rhs) {
      _FDiscreteFunction<T> tmp(rhs);
      *this = std::move(tmp);
      return *this;
    }
    /// \brief move ctor
    _FDiscreteFunction(_FDiscreteFunction<T>&& rhs) = default;
    /// \brief move assignment
    _FDiscreteFunction& operator=(_FDiscreteFunction&&) = default;

    ///
    /// \brief Assemble the flat table corresponding to this function
    /// \note Called after all state and action dependencies have been added
    ///
    virtual void pack(T initial = T(0)) {
      if(!this->_computed) {
          this->computeSubdomain();
      }
      // allocate memory
      _sa_table = boost::make_shared<_FStateActionTable<T>>(this->_subdomain, initial);
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
    /// \brief Return flat representation
    std::vector<T>& values() {
      assert(_packed);
      return _sa_table->values();
    }
    /// \brief Return flat representation
    const std::vector<T>& values() const {
      assert(_packed);
      return _sa_table->values();
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
    virtual bool eraseStateFactor(Size i) override {
        bool res = _DiscreteFunction<T>::eraseStateFactor(i);
        _packed = _DiscreteFunction<T>::_computed;
        return res;
    }
    virtual bool eraseActionFactor(Size i) override {
        bool res = _DiscreteFunction<T>::eraseActionFactor(i);
        _packed = _DiscreteFunction<T>::_computed;
        return res;
    }
    virtual void join(const SizeVec& state_dom, const SizeVec& action_dom, const LiftedVec& lifted_dom) override {
      _DiscreteFunction<T>::join(state_dom, action_dom, lifted_dom);
      _packed = _DiscreteFunction<T>::_computed;
    }
    virtual void join(const _DiscreteFunction<T>& func) override {
      _DiscreteFunction<T>::join(func);
      _packed = _DiscreteFunction<T>::_computed;
    }

    //
    // some other operations
    //

    /// \brief Multiply all values with a scalar
    _FDiscreteFunction<T>& operator*=(T s) {
      assert(_packed);
      auto& vals = _sa_table->values();
      std::transform(vals.begin(), vals.end(), vals.begin(), [s](T v) { return v*s; });
      return *this;
    }

    /// \brief Add another \a DiscreteFunction to this one (element-wise)
    /// Supports the case that function \a other is defined over a (proper) subset of factors from this function
    _FDiscreteFunction<T>& operator+=(const _FDiscreteFunction<T>& other) {
      assert(_packed);
      transform(&other, std::plus<T>(), true);
      return *this;
    }
    _FDiscreteFunction<T>& operator+=(const _DiscreteFunction<T>* other) {
      assert(_packed);
      transform(other, std::plus<T>(), false);
      return *this;
    }
    /// \brief Subtract another \a DiscreteFunction from this one (element-wise)
    /// Supports the case that function \a other is defined over a (proper) subset of factors from this function
    _FDiscreteFunction<T>& operator-=(const _FDiscreteFunction<T>& other) {
      assert(_packed);
      transform(&other, std::minus<T>(), true);
      return *this;
    }
    _FDiscreteFunction<T>& operator-=(const _DiscreteFunction<T>* other) {
      assert(_packed);
      transform(other, std::minus<T>(), false);
      return *this;
    }
};
template<class T>
using FDiscreteFunction = boost::shared_ptr<_FDiscreteFunction<T>>;

//
// Some useful functions
//

/**
 * \brief Indicator basis centered on a specific state
 */
template<class T = double>
class _Indicator : public _DiscreteFunction<T> {
protected:
  /// \brief The state on which this indicator function is centered
  Size _s_index;
public:
  /// \brief ctor
  _Indicator(const Domain& domain, std::string name = "")
  : _DiscreteFunction<T>(domain, name) { }
  /// \brief Initialize an indicator on \a State s in a subdomain
  /// \param factors Denotes a (sub-)set of state factors from the supplied \a Domain
  _Indicator(const Domain& domain, SizeVec factors, const State& s, std::string name = "")
  : _DiscreteFunction<T>(domain, name) {
    assert(s);
    std::sort(factors.begin(),factors.end());
    _DiscreteFunction<T>::join(factors, this->getActionFactors(), LiftedVec()); // no lifted dom support for now
    setState(s);
  }

  /// \brief Define the State for which this Indicator is set
  void setState(const State& s) {
    if(!_DiscreteFunction<T>::_computed) {
      _DiscreteFunction<T>::computeSubdomain();
    }
    _s_index = s; // copy index only-no other comparison (for identical domain, etc.) done
  }
  /// \brief Get the state (index) for which this Indicator is set
  Size getStateIndex() const {
      return _s_index;
  }

  virtual T eval(const State& s, const Action& a) const override {
    return static_cast<T>(_s_index == (Size)s); // note: only compares indices
  }
};
template<class T = double>
using Indicator = boost::shared_ptr<_Indicator<T>>;

/**
 * \brief A constant function
 */
template<class T = double>
class _ConstantFn : public _DiscreteFunction<T> {
protected:
  T _val;
public:
  _ConstantFn(const Domain& domain, T val = T(1), std::string name = "")
  : _DiscreteFunction<T>(domain, name), _val(val) {
      _DiscreteFunction<T>::computeSubdomain();
  }

  /// \brief Return 1 for any (s,a)
  virtual T eval(const State& s, const Action& a) const override {
    return _val;
  }
  /// \brief Multiply all values with a scalar
  _ConstantFn<T>& operator*=(T s) {
    _val *= s;
    return *this;
  }
  _ConstantFn<T>& operator+=(const _ConstantFn<T>& other) {
    _val += other._val;
    return *this;
  }
  _ConstantFn<T>& operator-=(const _ConstantFn<T>& other) {
    _val -= other._val;
    return *this;
  }
};
template<class T = double>
using ConstantFn = boost::shared_ptr<_ConstantFn<T>>;

} // namespace crl

//
// algorithm implementations
//
#include "algorithm.hpp"

#endif /*FUNCTION_HPP_*/
