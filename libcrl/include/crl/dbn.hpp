/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#ifndef DBN_HPP_
#define DBN_HPP_

#include <cassert>
#include "crl/crl.hpp"
#include "crl/flat_tables.hpp"
#include "crl/function.hpp"

namespace crl {

//typedef _StateActionTable<ProbabilityVec> _SAFProbTable;
//typedef boost::shared_ptr<_SAFProbTable> SAFProbTable;
//typedef _StateActionTable<Reward> _SAFRewardTable;
//typedef boost::shared_ptr<_SAFRewardTable> SAFRewardTable;

/**
 * \brief The definition of a single DBN factor across 2 time slices (t-1 -> t), that can have its dynamics set explicitly.
 *  Includes the transition probabilities encoded with tabular storage, i.e. (s,a) -> Pr(t)
 * \note All dependencies (delayed, concurrent, action) are sorted internally in ascending order.
 */
class _DBNFactor : public _FDiscreteFunction<ProbabilityVec> {
private:
  // trick to change visibility to private -- for DBN factors we need to be explicit about delayed (t-1) and concurrent (t) dependencies
  using _FDiscreteFunction::addStateFactor;
  using _FDiscreteFunction::eraseStateFactor;
  using _FDiscreteFunction::join;
  using _DiscreteFunction::containsStateFactor;
  using _DiscreteFunction::getStateFactors;
  using _DiscreteFunction::_state_dom;
  //using _DiscreteFunction::mapState;
protected:
  /// Denoting the (single) factor in the domain considered by this DBNFactor
  Size _target;
  /// The range of the (single) factor in the domain considered by this DBNFactor
  FactorRange _target_range;
  /// Denoting delayed dependencies of this factor, i.e., an edge from t to t+1 in the DBN.
  SizeVec _delayed_dep;
  /// Denoting concurrent dependencies of this factor, i.e., an edge from t+1 to t+1 in the DBN.
  SizeVec _concurrent_dep;
  /// \brief A dummy, empty state
  State _empty_s;
public:
  ///
  /// \brief Extract the relevant state information for this factor (i.e., those corresponding to this \a _subdomain)
  /// \param s The current (e.g., joint) state
  /// \param n The (e.g., joint) successor state (for example, from an \a Observation)
  /// \param domain_map_s An (optional) function mapping state factor Ids in the global _domain to those in the supplied \a States s
  /// \param domain_map_n An (optional) function mapping state factor Ids in the global _domain to those in the supplied \a States n
  /// \note Only available after call to \a pack()
  /// \note When the size of state s corresponds to size of local state space, s is directly returned (optimization)
  /// \note When the size of state s does not correspond to size of local state space, the provided \a domain_map is used for resolution.
  ///
  template <class T>
  State mapState(const State& s, const State& n, T&& domain_map_s, T&& domain_map_n) const {
      assert(_packed);
      if(s.size() == _delayed_dep.size() && !n) // under these conditions no reduction to local scope performed
          return s;
      State ms(_subdomain);
      for (Size i=0; i<_delayed_dep.size(); i++) {
          Size j = _delayed_dep[i];
          ms.setFactor(i, s.getFactor(domain_map_s(j)));
      }
      // append those factors corresponding to concurrent dependencies
      for (Size i=0; i<_concurrent_dep.size(); i++) {
          Size j = _concurrent_dep[i];
          ms.setFactor(i+_delayed_dep.size(), n.getFactor(domain_map_n(j)));
      }
      return ms;
  }
  State mapState(const State& s, const State& n) const {
    return mapState(s, n, identity_map, identity_map);
  }

  ///
  /// \brief Initialize this \a DBNFactor for a specific factor in the domain.
  ///
  _DBNFactor(const Domain& domain, Size target);

  void addDelayedDependency(Size index);
  const SizeVec& getDelayedDependencies() const { return _delayed_dep; }
  void addConcurrentDependency(Size index);
  const SizeVec& getConcurrentDependencies() const { return _concurrent_dep; }
  void addActionDependency(Size index);
  const SizeVec& getActionDependencies() const { return _action_dom; }
  bool hasConcurrentDependency() const { return !_concurrent_dep.empty(); }

  /// \brief Compute the domain of this DBNFactor
  /// \note This is overloaded from _DiscreteFunction to support both delayed and concurrent dependencies
  virtual void computeSubdomain() override;

  /// \brief Return the (number referring to the) factor in the domain that this DBNFactor represents
  Size getTarget() const {
    return _target;
  }
  /// \brief Return range of this factor
  const FactorRange& getTargetRange() const {
    return _target_range;
  }
  /// \brief Less-than operator overload to support sorting
  bool operator<(const _DBNFactor &rhs) const {
    return _target < rhs._target;
  }

  /// \brief The vector of probabilities for successor values associated with the tuple (s,n,a)
  /// \param state_map_s An (optional) function mapping state factor Ids in the global _domain to those in the supplied \a States s
  /// \param state_map_n An (optional) function mapping state factor Ids in the global _domain to those in the supplied \a States n
  /// \param action_map An (optional) function mapping action factor Ids in the global _domain to those in the supplied \a Action a
  template<class C>
  const ProbabilityVec& T(const State& s, const State& n, const Action& a, C&& state_map_s, C&& state_map_n, C&& action_map) const {
      State ms = mapState(s, n, std::forward<C>(state_map_s), std::forward<C>(state_map_n));
      Action ma = mapAction(a, std::forward<C>(action_map));
      ProbabilityVec& pv = _sa_table->getValue(ms, ma);
      return pv;
  }
  /// \note This particular function supports either joint state/action as parameters or state/action that are already at factor scope
  const ProbabilityVec& T(const State& s, const State& n, const Action& a) const {
      return T(s, n, a, identity_map, identity_map, identity_map);
  }
  /// \brief The probability of transitioning to a particular state value t from (s,n,a)
  /// \param state_map_s An (optional) function mapping state factor Ids in the global _domain to those in the supplied \a States s
  /// \param state_map_n An (optional) function mapping state factor Ids in the global _domain to those in the supplied \a States n
  /// \param action_map An (optional) function mapping action factor Ids in the global _domain to those in the supplied \a Action a
  template<class C>
  Probability T(const State& s, const State& n, const Action& a, Factor t, C&& state_map_s, C&& state_map_n, C&& action_map) const {
      const ProbabilityVec& pv = T(s, n, a, std::forward<C>(state_map_s), std::forward<C>(state_map_n), std::forward<C>(action_map));
      Factor offset = t - _target_range.getMin();
      return pv[offset];
  }
  /// \note This particular function supports either joint state/action as parameters or state/action that are already at factor scope
  Probability T(const State& s, const State& n, const Action& a, Factor t) const {
      return T(s, n, a, t, identity_map, identity_map, identity_map);
  }

  /// \brief Convenience function for the case that no concurrent dependencies exist in 2DBN
  /// \param state_map An (optional) function mapping state factor Ids in the global _domain to those in the supplied \a States s
  /// \param action_map An (optional) function mapping action factor Ids in the global _domain to those in the supplied \a Action a
  template<class C>
  const ProbabilityVec& T(const State& s, const Action& a, C&& state_map, C&& action_map) const {
    if(!_concurrent_dep.empty()) {
        throw cpputil::InvalidException("Transition function is missing state(t) to compute concurrent dependencies.");
    }
    return T(s, _empty_s, a, std::forward<C>(state_map), identity_map, std::forward<C>(action_map));
  }
  /// \note This particular function supports either joint state/action as parameters or state/action that are already at factor scope
  const ProbabilityVec& T(const State& s, const Action& a) const {
      return T(s, a, identity_map, identity_map);
  }

  ///
  /// \brief Set transition probability from (s,n,a) -> t, the target value of this factor
  /// \param s The (e.g, complete) state at t-1
  /// \param n The (e.g., complete) state at t
  /// \param a The (e.g., joint) action
  /// \param t The value of this DBN factor
  /// \param p The probability associated with this value
  /// \note Probabilities are not normalized inside this function
  /// \note This particular function supports either joint state/action as parameters or state/action that are already at factor scope
  ///
  void setT(const State& s, const State& n, const Action& a, Factor t, Probability p);
  /// \brief Convenience function for the case that no concurrent dependencies exist in 2DBN
  /// \note This particular function supports either joint state/action as parameters or state/action that are already at factor scope
  void setT(const State& s, const Action& a, Factor t, Probability p) {
    if(!_concurrent_dep.empty()) {
        throw cpputil::InvalidException("Transition function is missing state(t) to compute concurrent dependencies.");
    }
    setT(s,_empty_s,a,t,p);
  }
};

typedef boost::shared_ptr<_DBNFactor> DBNFactor;
// iterators
typedef cpputil::Iterator<DBNFactor> _FactorIterator;
typedef boost::shared_ptr<_FactorIterator> FactorIterator;
typedef cpputil::VectorIterator<DBNFactor> _FactorVecIterator;
typedef boost::shared_ptr<_FactorIterator> FactorVecIterator;

/**
 * \brief A Local Reward Function (LRF) is just a function that returns rewards
 * Rewards are defined as the mapping (s,a) -> r where (s,a) denotes the current state and action.
 * \note Concurrent dependencies (i.e., rewards depending on successor state n) are not (explicitly) supported
 */
class _LRF : public _FDiscreteFunction<Reward> {
public:
  _LRF(const Domain& domain)
  : _FDiscreteFunction(domain) { }

  /// \brief The reward associated with the tuple (s,a)
  /// \note This particular function supports either joint state/action as parameters or state/action that are already at factor scope
  Reward R(const State& s, const Action& a) const;
  /// \brief Set the (local) reward associated with the tuple (s,a)
  /// \note This particular function supports either joint state/action as parameters or state/action that are already at factor scope
  void setR(const State& s, const Action& a, Reward r);
};
typedef boost::shared_ptr<_LRF> LRF;

/**
 * \brief The 2-stage DBN (2-TBN) encoding the transition function from t-1 to t.
 * The 2-TBN supports factored states and actions.
 * \note DBNFactors are sorted internally in ascending order of their target variables.
 */
class _DBN {
protected:
  std::vector<DBNFactor> _dbn_factors;
  /// \brief True iff there are any concurrent dependencies in the DBN (i.e., at time slice t)
  bool _has_concurrency;
public:
  _DBN()
  : _has_concurrency(false) { }

  /// \brief Return the number of \a DBNFactors in this DBN
  Size size() const {
    return _dbn_factors.size();
  }

  /// \brief Add a \a DBNFactor to this \a DBN
  /// \note Overwrites an existing DBNFactor with the identical target variable
  void addDBNFactor(DBNFactor dbn_factor);
  /// \brief Iterator over all \a DBNFactor in this DBN
  FactorIterator factors() {
    return boost::make_shared<_FactorVecIterator>(_dbn_factors);
  }
  /// \brief Return \a DBNFactor for a specific target variable
  DBNFactor factor(Size t) const {
    assert(t < size());
    return _dbn_factors[t];
  }
  /// \brief True iff there are any concurrent dependencies in the DBN (at time slice t)
  bool hasConcurrentDependency() const {
    return _has_concurrency;
  }

  /// \brief Compute the probability of transitioning from (joint) s -> n under (joint) \a Action a
  Probability T(const State& js, const Action& ja, const State& jn);
  /// \brief Compute the probability of transitioning from (sub_s,sub_a) -> n
  /// As per the 2-TBN definition, this is a product over a selection of \a DBNFactors.
  template<class C>
  Probability T(const State& s, const Action& a, const State& n, C state_map_s, C state_map_n, C action_map) {
      Probability p = 1.;

      const auto& targets = state_map_n.map();
      for(auto kv : targets) {
          //kv.first: the global factor Id (for DBN)
          const DBNFactor& f = factor(kv.first);
          //kv.second: the local mapping in the given State n
          Factor t = n.getFactor(kv.second);
          p *= f->T(s, n, a, t, state_map_s, state_map_n, action_map);
      }
      return p;
  }
};
typedef boost::shared_ptr<_DBN> DBN;

/// \brief Template specialization corresponding to total (joint) probability transitioning from s -> under (joint) \a Action a
template<> Probability _DBN::T(const State& js, const Action& ja, const State& jn, decltype(identity_map), decltype(identity_map), decltype(identity_map)) {
    return _DBN::T(js,ja,jn);
}

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
    assert(_FDiscreteFunction<T>::_packed && _cached);
    return _FDiscreteFunction<T>::eval(s,a);
  }

  /// \brief Multiply all values with a scalar
  _Backprojection<T>& operator*=(T s) {
    assert(_FDiscreteFunction<T>::_packed && _cached);
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

}

#endif /*DBN_HPP_*/
