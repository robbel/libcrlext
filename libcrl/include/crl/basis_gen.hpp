/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#ifndef BASIS_GEN_HPP_
#define BASIS_GEN_HPP_

#include <unordered_set>
#include <boost/functional/hash.hpp>
#include "crl/alp.hpp"

//
// Iterative basis construction
//

namespace crl {

// Forward declarations, typedefs
class Conjunction;
template<class T> class _FConjunctiveFeature;
template<class T> class _BinConjunctiveFeature;
template<class T> std::ostream& operator<<(std::ostream& os, const _FConjunctiveFeature<T>& f);
template<class T> std::ostream& operator<<(std::ostream& os, const _BinConjunctiveFeature<T>& f);

/// \brief NChooseTwoIterator for iterating over \a SizeVec
typedef cpputil::NChooseTwoIterator<Size, SizeVec> _SizeChooseTwoIterator;
typedef boost::shared_ptr<_SizeChooseTwoIterator> SizeChooseTwoIterator;

namespace algorithm {

template<class T, class BinOp = decltype(std::plus<T>())>
DiscreteFunction<T> pair(const Conjunction& joint_base, const DiscreteFunction<T>& h1, const DiscreteFunction<T>& h2, BinOp binOp);
template<class T, class BinOp = decltype(std::plus<T>())>
DiscreteFunction<T> pair(Size h1_id, Size h2_id, const DiscreteFunction<T>& h1, const DiscreteFunction<T>& h2, BinOp binOp);
template<class T> Conjunction pair_basis(Size h1_id, Size h2_id, const DiscreteFunction<T>& h1, const DiscreteFunction<T>& h2);
template<class T> DiscreteFunction<T> binpair(Size h1_id, Size h2_id, const DiscreteFunction<T>& h1, const DiscreteFunction<T>& h2);
template<class T> DiscreteFunction<T> binconj(const SizeVec& ids, const FactoredValueFunction& vfn);
template<class T, class BinRefOp> T evalOpOverBasis(const _DiscreteFunction<T>* basis, const FactoredFunction<T>& facfn, bool known_flat, T init, BinRefOp binOp);

}

/**
 * \brief Stores the (flat) conjunction of original basis functions that make up a conjunctive feature
 */
class Conjunction {
  template<class T> friend Conjunction algorithm::pair_basis(Size h1_id, Size h2_id, const DiscreteFunction<T>& h1, const DiscreteFunction<T>& h2);
  template<class T> friend DiscreteFunction<T> algorithm::binconj(const SizeVec& ids, const FactoredValueFunction& vfn);
  template<class F, class T, class BinOp> friend struct algorithm::join_Impl;
  template<class F, class T, class BinRefOp> friend struct algorithm::genericOp_Impl;
protected:
  /// \brief The (sorted) index set of basis functions that make up this conjunction
  SizeVec _base_fns;
  /// \brief Hash of _base_fns for checking uniqueness of feature
  std::size_t _hash;
public:
  /// \brief Default hash for empty conjunction
  static const std::size_t EMPTY_HASH;
  /// \brief ctor
  Conjunction()
  : _hash(EMPTY_HASH) { }
  /// \brief Return the (sorted) index set of basis functions that make up this conjunction
  const SizeVec& getBaseFeatures() const {
      return _base_fns;
  }
  /// \brief Return unique hash of this conjunction
  std::size_t getHash() const {
    return _hash;
  }
  /// \brief Return unique hash of this conjunction
  operator std::size_t() const {
    return _hash;
  }
  /// \brief operators
  bool operator==(const Conjunction& other) const {
    return _hash == other._hash;
  }
  bool operator!=(const Conjunction& other) const {
    return !(*this == other);
  }
  /// \brief The number of basis functions that make up this conjunction
  const Size size() {
    return _base_fns.size();
  }
};

/// \brief Stream output of a \a Conjunction
std::ostream& operator<<(std::ostream &os, const Conjunction& f);

/**
 * \brief A conjunctive feature with tabular storage
 * Internally, this feature is just a tabular discrete function that keeps track of
 * the (flat) conjunction of original basis functions that make up this feature.
 */
template<class T>
class _FConjunctiveFeature : public _FDiscreteFunction<T>, public Conjunction {
  // C++ does not allow partial specialization of template friends; hence specified for types U, BinOp
  template<class U, class BinOp>
  friend DiscreteFunction<U> algorithm::pair(const Conjunction& joint_base, const DiscreteFunction<U>& h1, const DiscreteFunction<U>& h2, BinOp binOp);
public:
  /// \brief ctor
  _FConjunctiveFeature(const Domain& domain, std::string name = "")
  : _FDiscreteFunction<T>(domain, name) { }
  /// \brief other ctors
  _FConjunctiveFeature(const _FConjunctiveFeature<T>& rhs) = default;
  _FConjunctiveFeature(_FConjunctiveFeature<T>&& rhs) = default;
  /// \brief move ctor from base _FDiscreteFunction
  _FConjunctiveFeature(_FDiscreteFunction<T>&& rhs)
  : _FDiscreteFunction<T>(std::move(rhs)) { }
  /// \brief copy ctor from base _FDiscreteFunction
  _FConjunctiveFeature(const _FDiscreteFunction<T>& rhs)
  : _FDiscreteFunction<T>(rhs) { }
  /// \brief assignment ops
  _FConjunctiveFeature& operator=(const _FConjunctiveFeature& rhs) = default;
  _FConjunctiveFeature& operator=(_FConjunctiveFeature&&) = default;
};
template<class T>
using FConjunctiveFeature = boost::shared_ptr<_FConjunctiveFeature<T>>;

/// \brief Stream output of a \a FConjunctiveFeature
template<class T>
std::ostream& operator<<(std::ostream &os, const _FConjunctiveFeature<T>& f) {
  os << static_cast<const Conjunction&>(f);
  os << ": S{";
  for(auto s : f.getStateFactors()) {
    os << s << ", ";
  }
  os << "})";
  return os;
}

/**
 * \brief A binary conjunctive feature with \a _Indicator storage
 * Internally, this feature is just an Indicator function that keeps track of
 * the (flat) conjunction of original basis functions that make up this feature.
 */
template<class T>
class _BinConjunctiveFeature : public _Indicator<T>, public Conjunction {
  friend DiscreteFunction<T> algorithm::binpair<T>(const Conjunction& joint_base, const DiscreteFunction<T>& h1, const DiscreteFunction<T>& h2);
public:
  /// \brief ctor
  _BinConjunctiveFeature(const Domain& domain, std::string name = "")
  : _Indicator<T>(domain, name) { }
};
template<class T>
using BinConjunctiveFeature = boost::shared_ptr<_BinConjunctiveFeature<T>>;

/// \brief Stream output of a \a BinConjunctiveFeature
template<class T>
std::ostream& operator<<(std::ostream &os, const _BinConjunctiveFeature<T>& f) {
  os << static_cast<const Conjunction&>(f);
  os << ": S{";
  for(auto s : f.getStateFactors()) {
    os << s << ", ";
  }
  os << "})";
  return os;
}

/**
 * \brief Base class for different basis function scoring implementations
 */
class BasisScore {
protected:
  /// \brief The (global) domain
  Domain _domain;
  /// \brief The (solved) factored value function
  const FactoredValueFunction& _value_fn;
public:
  /// \brief ctor
  /// \param vfn The (solved) factored value function that will be used for scoring
  BasisScore(const Domain& domain, const FactoredValueFunction& vfn, const FactoredMDP& fmdp)
  : _domain(domain), _value_fn(vfn) { }
  /// \brief Called once before every (complete) scoring run over candidate basis functions
  virtual void initialize() { }
  /// \brief The (abstract) scoring function for the given basis function
  virtual double score(const DiscreteFunction<Reward>& basis) const = 0;
  /// \brief The name of this scoring function
  virtual std::string getName() const = 0;
};

/**
 * \brief Computing 1/2*(max BE(f) - min BE(f)) over the region where the feature f is active
 */
class EpsilonScore : public BasisScore {
public:
  /// \brief ctor
  /// \param vfn The (solved) factored value function that will be used for scoring
  EpsilonScore(const Domain& domain, const FactoredValueFunction& vfn, const FactoredMDP& fmdp)
  : BasisScore(domain, vfn, fmdp) { }
  /// \brief score implementation
  double score(const DiscreteFunction<Reward>& basis) const;
  std::string getName() const {
    return "EpsilonScore";
  }
};

/**
 * \brief Computing 1/2*(max BE(f) + min BE(f)) over the region where the feature f is active
 */
class AbsoluteReductionScore : public BasisScore {
public:
  /// \brief ctor
  /// \param vfn The (solved) factored value function that will be used for scoring
  AbsoluteReductionScore(const Domain& domain, const FactoredValueFunction& vfn, const FactoredMDP& fmdp)
  : BasisScore(domain, vfn, fmdp) { }
  /// \brief score implementation
  double score(const DiscreteFunction<Reward>& basis) const;
  std::string getName() const {
    return "AbsoluteReductionScore";
  }
};

/**
 * \brief Computing the BEBF score Sum_BE(f) / sqrt(|Coverage(f)|) over the region where feature f is active
 */
class BEBFScore : public BasisScore {
protected:
  /// \brief Cached copy of the maxQ function
  std::vector<DiscreteFunction<Reward>> _maxQ;
public:
  /// \brief ctor
  /// \param vfn The (solved) factored value function that will be used for scoring
  BEBFScore(const Domain& domain, const FactoredValueFunction& vfn, const FactoredMDP& fmdp)
  : BasisScore(domain, vfn, fmdp) { }
  virtual void initialize();
  /// \brief score implementation
  virtual double score(const DiscreteFunction<Reward>& basis) const;
  virtual std::string getName() const {
    return "BEBFScore";
  }
};

/**
 * \brief Computing the BEBF score Sum_BE(f) / sqrt(|Coverage(f)|) over the region where feature f is active
 * Additionally enforcing sparsity in factor graph to allow (exact) BE computation
 */
class OptBEBFScore : public BEBFScore {
protected:
  static const Size DEFAULT_MAX_SCOPE;
  /// \brief The maximum supported scope size in the `action connected' partition of the factor graph
  Size _max_scope;
  /// \brief The \a DBN used for backprojections
  const _DBN& _dbn;
public:
  /// \brief ctor
  /// \param vfn The (solved) factored value function that will be used for scoring
  OptBEBFScore(const Domain& domain, const FactoredValueFunction& vfn, const FactoredMDP& fmdp)
  : BEBFScore(domain, vfn, fmdp), _max_scope(DEFAULT_MAX_SCOPE), _dbn(fmdp->T()) { }
  virtual void initialize();
  Size getMaxScope() const {
    return _max_scope;
  }
  void setMaxScope(Size sc) {
    _max_scope = sc;
  }
  /// \brief score implementation
  virtual double score(const DiscreteFunction<Reward>& basis) const;
  virtual std::string getName() const {
    return "OptBEBFScore(" + std::to_string(_max_scope) + ")";
  }
};

/**
 * \brief A BinaryBasisGenerator uses a scoring mechanism to compute the next best indicator basis
 * Evaluates (binary) indicator features based on conjunctions of existing features and the supplied scoring method
 * \tparam It The iterator to compute new candidate basis Id-tuples
 * \tparam Sc The type of the scoring method
 * \see cpputil::NChooseTwoIterator, BasisScore
 */
template<class It, class Sc>
class BinaryBasisGenerator {
protected:
  /// \brief The (global) domain
  Domain _domain;
  /// \brief The (solved) factored value function
  FactoredValueFunction _value_fn;
  /// \brief The \a BasisScore implementation to score each candidate basis
  Sc _score;
  /// \brief Unique name of this basis generator
  std::string _name;
  /// \brief Hash-Set to keep track of existing features
  std::unordered_multiset<Conjunction,std::hash<std::size_t>> _exists;
public:
  /// \brief ctor
  BinaryBasisGenerator(const Domain& domain, FactoredValueFunction vfn, FactoredMDP fmdp, std::string name = "")
  : _domain(domain), _value_fn(std::move(vfn)), _score(_domain, _value_fn, std::move(fmdp)), _name(name) { }
  /// \brief Clear the feature cache
  void clearCache() {
    _exists.clear();
  }
  /// \brief Return the next best basis function
  DiscreteFunction<Reward> nextBest() {
    // initialize the scoring function
    _score.initialize();
    // Note: sort by size?
    LOG_DEBUG("Basis size: " << _value_fn->getBasis().size());
    const auto& basisVec = _value_fn->getBasis();
    SizeVec basisIds = cpputil::ordered_vec<Size>(basisVec.size());
    It biter(basisIds);

    DiscreteFunction<Reward> bestf;
    double bestVal = -std::numeric_limits<double>::infinity();
    Size count = 0, valid = 0;
    while(biter.hasNext()) {
        const auto& tpl = biter.next();
        count++;
        Size h1_id = std::get<0>(tpl);
        Size h2_id = std::get<1>(tpl);
        Conjunction joint_base = algorithm::pair_basis(h1_id, h2_id, basisVec[h1_id], basisVec[h2_id]);

        // test whether this conjunctive feature was already checked
        auto range = _exists.equal_range(joint_base);
        if(range.first != range.second) {
            // test for exact equality versus hash collision
            auto equalFn = [&joint_base](const Conjunction& c2) { return joint_base.getBaseFeatures() == c2.getBaseFeatures(); };
            auto cit = std::find_if(range.first, range.second, equalFn);
            if(cit != range.second) {
              LOG_DEBUG("Skipping " << joint_base << ": already checked this one: " << *cit
                        << " which has hash: " << cit->getHash());
              continue;
            }
        }
        // update checked hashes
        _exists.insert(joint_base);

        // perform a logical `AND' to obtain new joint indicator
        DiscreteFunction<Reward> candf = algorithm::binpair(joint_base, basisVec[h1_id], basisVec[h2_id]);
        if(!candf) { // no consistent indicator exists for h1 ^ h2
            continue;
        }
        // score candidate function
        double s = _score.score(candf);
        LOG_DEBUG("<" << h1_id << "," << h2_id << ">: " << s);
        if(s > bestVal) {
          bestf = std::move(candf);
          bestVal = s;
        }
        valid++;
    }

    if(bestf) {
      const Conjunction* bestc = dynamic_cast<const Conjunction*>(bestf.get());
      LOG_INFO("Best next function (score: " << bestVal << "): " << *bestf << " which is: " << *bestc);
      LOG_INFO("Evaluated " << valid << " valid conjunctive features out of " << count << ".");
    }
    return bestf;
  }
};

//
// Algorithms for basis function evaluation
//

namespace {

// helper function for algorithm::pair
template<class It, class Ins>
inline void pair_helper(It it1b, It it1e, const Conjunction* cf2, const Size (&h2_arr)[1], Ins& ins) {
  if(cf2)
      std::set_union(it1b, it1e, std::begin(cf2->getBaseFeatures()), std::end(cf2->getBaseFeatures()), ins);
  else
      std::set_union(it1b, it1e, std::begin(h2_arr), std::end(h2_arr), ins);
}

} // anonymous ns

namespace algorithm {

/// \brief Compute the (flat, ordered) union of original basis functions that make up the joint feature
/// \return The conjunction of basis functions
template<class T>
Conjunction pair_basis(Size h1_id, Size h2_id, const DiscreteFunction<T>& h1, const DiscreteFunction<T>& h2) {
  // compute basis features that are active in conjunction
  Conjunction joint_base;
  const Conjunction* cf1 = dynamic_cast<const Conjunction*>(h1.get());
  const Conjunction* cf2 = dynamic_cast<const Conjunction*>(h2.get());
  Size h1_arr[] = { h1_id };
  Size h2_arr[] = { h2_id };
  // do std::set_union in canonical form for all instances instead of using, e.g., boost::any_range
  auto ins = std::back_inserter(joint_base._base_fns);
  if(cf1) {
      pair_helper(std::begin(cf1->getBaseFeatures()), std::end(cf1->getBaseFeatures()), cf2, h2_arr, ins);
  }
  else {
      pair_helper(std::begin(h1_arr), std::end(h1_arr), cf2, h2_arr, ins);
  }
  // compute the hash for the conjunction
  joint_base._hash = boost::hash_range(joint_base._base_fns.begin(), joint_base._base_fns.end());
  return joint_base;
}

/// \brief Compute the conjunction (with a given operation) of two features defined over state factors
/// This is a specialization of the general algorithm::join to obtain a FConjunctiveFeature
/// \param joint_base The (flat, ordered) conjunction of original basis functions that make up joint feature
/// \param binOp The operation to perform when joining features (e.g., sum)
/// \see algorithm::join, algorithm::pair_basis
template<class T, class BinOp>
DiscreteFunction<T> pair(const Conjunction& joint_base, const DiscreteFunction<T>& h1, const DiscreteFunction<T>& h2, BinOp binOp) {
  assert(h1 && h2 && h1->getActionFactors().empty() && h2->getActionFactors().empty());
  // TODO: add optimization path when h1, h2 subdomains are identical/subsets, see _FDiscreteFunction::transform
  DiscreteFunction<T> jf = algorithm::join<_FConjunctiveFeature<T>>({h1,h2}, binOp);
  _FConjunctiveFeature<T>* pcf = static_cast<_FConjunctiveFeature<T>*>(jf.get());
  // update basis features that are active in conjunction
  pcf->_base_fns = joint_base.getBaseFeatures();
  pcf->_hash = joint_base.getHash();
  return jf;
}

/// \brief Compute the binary conjunction (`AND') of two binary features defined over state factors
/// Both h1 and h2 are assumed to be binary indicator functions
/// \param joint_base The (flat, ordered) conjunction of original basis functions that make up joint feature
/// \return A \a _BinConjunctiveFeature wrapped as a DiscreteFunction. Nullptr if no joint indicator exists.
/// \see crl::_Indicator, algorithm::pair_basis
template<class T>
DiscreteFunction<T> binpair(const Conjunction& joint_base, const DiscreteFunction<T>& h1, const DiscreteFunction<T>& h2) {
  assert(h1 && h2 && h1->getActionFactors().empty() && h2->getActionFactors().empty());
  // join function scopes
  BinConjunctiveFeature<T> jb = boost::make_shared<_BinConjunctiveFeature<T>>(h1->_domain);
  jb->join(*h1.get());
  jb->join(*h2.get());
  jb->computeSubdomain();

  // compute single active state where both indicators are jointly active
  const _Indicator<T>* i1 = dynamic_cast<const _Indicator<T>*>(h1.get());
  const _Indicator<T>* i2 = dynamic_cast<const _Indicator<T>*>(h2.get());
  assert(i1 && i2);
  const State i1_s = State(i1->getSubdomain(),i1->getStateIndex());
  const SizeVec& i1_sdom = i1->getStateFactors();
  const State i2_s = State(i2->getSubdomain(),i2->getStateIndex());
  const SizeVec& i2_sdom = i2->getStateFactors();

  // build a flat representation of jointly enabled state
  const Factor unset = std::numeric_limits<Factor>::min();
  std::vector<Factor> js_flat(jb->getStateFactors().size(), unset);
  subdom_map js_dom(jb->getStateFactors());
  for(Size i = 0; i < i1_sdom.size(); i++) {
      js_flat[js_dom(i1_sdom[i])] = i1_s.getFactor(i);
  }
  for(Size j = 0; j < i2_sdom.size(); j++) {
      // required indirection to pick out correct position in js_flat
      auto& js_entry = js_flat[js_dom(i2_sdom[j])];
      if(js_entry == unset) {
        js_entry = i2_s.getFactor(j);
      }
      else if(js_entry != i2_s.getFactor(j)) { // conflicting values for shared variable
        js_entry = unset;
      }
  }
  // check for incompatible indicator functions
  if(std::find(begin(js_flat), end(js_flat), unset) != end(js_flat)) {
      return nullptr;
  }
  // fill joint state
  State js(jb->getSubdomain());
  for(Size i = 0; i < jb->getStateFactors().size(); i++) {
      js.setFactor(i, js_flat[i]);
  }
  jb->setState(js);

  // update basis features that are active in conjunction
  jb->_base_fns = joint_base.getBaseFeatures();
  jb->_hash = joint_base.getHash();
  return jb;
}

/// \brief Compute the conjunction (with a given operation) of two features defined over state factors
/// This is a specialization of the general algorithm::join to obtain a FConjunctiveFeature
/// \param h1_id The unique Id of function h1 (e.g., position in basis vector)
/// \param h2_id The unique Id of function h2 (e.g., position in basis vector)
/// \param binOp The operation to perform when joining features (e.g., sum)
/// \see algorithm::join
template<class T, class BinOp>
DiscreteFunction<T> pair(Size h1_id, Size h2_id, const DiscreteFunction<T>& h1, const DiscreteFunction<T>& h2, BinOp binOp) {
  // compute basis features that are active in conjunction
  Conjunction joint_base = pair_basis(h1_id, h2_id, h1, h2);
  return pair(joint_base, h1, h2, binOp);
}

/// \brief Compute the binary conjunction (`AND') of two binary features defined over state factors
/// Both h1 and h2 are assumed to be binary indicator functions
/// \param h1_id The unique Id of indicator function h1 (e.g., position in basis vector)
/// \param h2_id The unique Id of indicator function h2 (e.g., position in basis vector)
/// \return A \a _BinConjunctiveFeature wrapped as a DiscreteFunction. Nullptr if no joint indicator exists.
/// \see crl::_Indicator, algorithm::pair_basis
template<class T>
DiscreteFunction<T> binpair(Size h1_id, Size h2_id, const DiscreteFunction<T>& h1, const DiscreteFunction<T>& h2) {
  // compute basis features that are active in conjunction
  Conjunction joint_base = pair_basis(h1_id, h2_id, h1, h2);
  return binpair(joint_base, h1, h2);
}

/// \brief Compute the binary conjunction (`AND') of the given binary features
/// \param ids The unique Ids of the indicator functions (e.g., positions in basis vector)
/// \return A \a _BinConjunctiveFeature wrapped as a DiscreteFunction. Nullptr if no joint indicator exists.
/// \note Inefficient implementation that merges all basis functions pairwise
/// \see crl::_Indicator, algorithm::binpair
template<class T>
DiscreteFunction<T> binconj(const SizeVec& ids, const FactoredValueFunction& vfn) {
  if(ids.size() < 2) {
      return nullptr;
  }
  // compute basis features that are active in conjunction
  Conjunction joint_base;
  joint_base._base_fns = ids;
  joint_base._hash = boost::hash_range(joint_base._base_fns.begin(), joint_base._base_fns.end());
  // merge all basis functions pairwise
  DiscreteFunction<T> conj = binpair(joint_base, vfn->getBasis()[ids[0]], vfn->getBasis()[ids[1]]);
  for(int i = 2; i < ids.size(); i++) {
      conj = binpair(joint_base, conj, vfn->getBasis()[i]);
      if(!conj) {
        break;
      }
  }
  return conj;
}

/// \brief Perform a given operation (e.g., max, min, sum) on the \a FactoredFunction for all values where the basis is `active'
/// \tparam BinRefOp A type [](T& v1, T& v2) -> void
/// \note Assumes that the FactoredFunction is defined over the same domain as `basis'
template<class T, class BinRefOp>
T evalOpOverBasis(const _DiscreteFunction<T>* basis, const FactoredFunction<T>& facfn, bool known_flat, T init, BinRefOp binOp) {
  assert(basis->getActionFactors().empty());
  // optimization path for basis functions with tabular storage
  const _FDiscreteFunction<T>* of = is_flat(basis, known_flat);
  const std::vector<T>* pvals = nullptr;
  typename std::vector<T>::size_type j = 0;
  if(of) {
      pvals = &of->values();
  }

  // obtain value of facfn empty functionals (shared for all states in basis' subdomain)
  T common_val = T(0);
  for(const auto& empty_fn : std::get<1>(facfn)) {
      common_val += empty_fn->eval(State(),Action());
  }

  // manually loop over basis function domain and apply binOp
  // Note: for variable elimination cases (min, max), I could continue to run that instead!
  const subdom_map s_map(basis->getStateFactors());
  T val = init;
  const _Indicator<T>* pI = dynamic_cast<const _Indicator<T>*>(basis);
  if(pI) {
      // basis is active in exactly one state
      const State s(basis->getSubdomain(), pI->getStateIndex());
      val = common_val;
      for(const auto& f : std::get<0>(facfn)) {
          State ms = f->mapState(s,s_map);
          val += f->eval(ms,Action());
      }
  }
  else {
      _StateIncrementIterator sitr(basis->getSubdomain());
      while(sitr.hasNext()) {
          const State& s = sitr.next();
          T basisVal = of ? (*pvals)[j++] : (*basis)(s,Action());
          if(basisVal != T(0)) { // basis is active in `s'
              T v = common_val; // candidate value
              for(const auto& f : std::get<0>(facfn)) {
                  State ms = f->mapState(s,s_map);
                  v += f->eval(ms,Action());
              }
              binOp(val, v);
          }
      }
  }
  return val;
}

/// \brief Compute a feature f's |Coverage(f)| with respect to the global state space.
/// \note The coverage of a feature can be zero if it is nowhere active!
// TODO: Size return ok?
template<class T>
Size basisCoverage(const Domain& domain, const _DiscreteFunction<T>* basis) {
  assert(!domain->isBig() && basis->getActionFactors().empty());
  const Size active_dom = domain->getNumStates()/basis->getSubdomain()->getNumStates();
  Size mul = 1;

  // an indicator is active in exactly one state
  const _Indicator<T>* pI = dynamic_cast<const _Indicator<T>*>(basis);
  if(!pI) {
      const _FDiscreteFunction<T>* ff = is_flat(basis, false);
      if(ff) {
          mul = std::count_if(ff->_sa_table->values().begin(), ff->_sa_table->values().end(),
                              [](T i) { return i != T(0); });
      } else {
          mul = 0;
          _StateIncrementIterator sitr(basis->getSubdomain());
          while(sitr.hasNext()) {
              const State& s = sitr.next();
              T basisVal = (*basis)(s,Action());
              if(basisVal != T(0)) { // basis is active in `s'
                  mul++;
              }
          }
      }
  }
  return active_dom * mul;
}

/// \brief Template specialization of genericOp for \a _FConjunctiveFeature
/// Additionally keeps track in a \a Conjunction of the underlying variables that are being operated on (e.g., summed out)
template<class T, class BinRefOp>
struct genericOp_Impl<_FConjunctiveFeature<T>, T, BinRefOp> {
  DiscreteFunction<T> operator()(const _DiscreteFunction<T>* pf, SizeVec vars, bool known_flat, T init, BinRefOp binOp) {
    // apply genericOp function
    DiscreteFunction<T> opf = algorithm::genericOp_Shared<_FConjunctiveFeature<T>>(pf, vars, known_flat, init, binOp);

    // update list of action variables that are being removed from this function
    _FConjunctiveFeature<T>* popf = static_cast<_FConjunctiveFeature<T>*>(opf.get());
    const _FConjunctiveFeature<T>* pinp = static_cast<const _FConjunctiveFeature<T>*>(pf);
    popf->_base_fns = pinp->getBaseFeatures();
    // hashing not used here
    //popf->_hash = boost::hash_range(popf->_base_fns.begin(), popf->_base_fns.end());
    return opf;
  }
};

/// \brief Template specialization of join for \a _FConjunctiveFeature
/// Additionally keeps track in a \a Conjunction of the underlying variables that are being operated on (e.g., summed out)
template<class T, class BinOp>
struct join_Impl<_FConjunctiveFeature<T>, T, BinOp> {
  /// \brief Join together the action scopes of the given functions into a \a Conjunction
  Conjunction pair_scope(cpputil::Iterator<DiscreteFunction<T>>& funcs) {
    // compute actions that are active in conjunction
    Conjunction joint_scope;
    auto& scope_vec = joint_scope._base_fns;
    while(funcs.hasNext()) {
        const auto& curFn = funcs.next();
        const Conjunction* cf = dynamic_cast<const Conjunction*>(curFn.get());
        const SizeVec& a_scope = cf ? cf->getBaseFeatures() : curFn->getActionFactors();
        scope_vec.insert(scope_vec.end(), a_scope.begin(), a_scope.end());
    }
    std::sort(scope_vec.begin(),scope_vec.end());
    scope_vec.erase(std::unique(scope_vec.begin(), scope_vec.end()), scope_vec.end());
    // hashing not used here
    //joint_scope._hash = boost::hash_range(joint_scope._base_fns.begin(), joint_scope._base_fns.end());
    return joint_scope;
  }

  DiscreteFunction<T> operator()(cpputil::Iterator<DiscreteFunction<T>>& funcs, BinOp binOp) {
    // apply join function
    DiscreteFunction<T> opf = join_Shared<_FConjunctiveFeature<T>>(funcs, binOp);

    // update list of action variables that are being joined by this function
    _FConjunctiveFeature<T>* popf = static_cast<_FConjunctiveFeature<T>*>(opf.get());
    funcs.reset();
    popf->_base_fns = pair_scope(funcs)._base_fns;
    // hashing not used here
    return opf;
  }
};

} // namespace algorithm

} // namespace crl

#endif /*BASIS_GEN_HPP_*/
