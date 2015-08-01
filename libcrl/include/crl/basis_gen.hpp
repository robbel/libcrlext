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
namespace algorithm {
template<class T, class BinOp = decltype(std::plus<T>())>
DiscreteFunction<T> pair(const SizeVec& joint_base, const DiscreteFunction<T>& h1, const DiscreteFunction<T>& h2, BinOp binOp);
template<class T, class BinOp = decltype(std::plus<T>())>
DiscreteFunction<T> pair(Size h1_id, Size h2_id, const DiscreteFunction<T>& h1, const DiscreteFunction<T>& h2, BinOp binOp);
template<class T, class BinOp> T evalOpOverBasis(const _DiscreteFunction<T>* basis, const FactoredFunction<T>& facfn, bool known_flat, T init, BinOp binOp);
}

template<class T> class _ConjunctiveFeature;
template<class T> std::ostream& operator<<(std::ostream& os, const _ConjunctiveFeature<T>& f);

/// \brief NChooseTwoIterator for iterating over \a SizeVec
typedef cpputil::NChooseTwoIterator<Size, SizeVec> _SizeChooseTwoIterator;
typedef boost::shared_ptr<_SizeChooseTwoIterator> SizeChooseTwoIterator;


/**
 * \brief A conjunctive feature is generated by `AND'ing other features
 * Internally, this feature is just a tabular discrete function but additionally keeps track of
 * the (flat) conjunction of original basis functions that make up this feature.
 */
template<class T>
class _ConjunctiveFeature : public _FDiscreteFunction<T> {
  // C++ does not allow partial specialization of template friends; hence specified for types U, BinOp
  template<class U, class BinOp>
  friend DiscreteFunction<U> algorithm::pair(const SizeVec& joint_base, const DiscreteFunction<U>& h1, const DiscreteFunction<U>& h2, BinOp binOp);
protected:
  /// \brief The (sorted) index set of basis functions that make up this conjunctive feature
  SizeVec _base_fns;
  /// \brief Hash of _base_fns for checking uniqueness of feature
  std::size_t _hash;
public:
  /// \brief Default hash for empty ConjunctiveFeature
  static const std::size_t EMPTY_HASH;
  /// \brief ctor
  _ConjunctiveFeature(const Domain& domain, std::string name = "")
  : _FDiscreteFunction<T>(domain, name), _hash(EMPTY_HASH) { }
  /// \brief other ctors
  _ConjunctiveFeature(const _ConjunctiveFeature<T>& rhs) = default;
  _ConjunctiveFeature(_ConjunctiveFeature<T>&& rhs) = default;
  /// \brief move ctor from base _FDiscreteFunction
  _ConjunctiveFeature(_FDiscreteFunction<T>&& rhs)
  : _FDiscreteFunction<T>(std::move(rhs)), _hash(EMPTY_HASH) { }
  /// \brief assignment ops
  _ConjunctiveFeature& operator=(const _ConjunctiveFeature& rhs) = default;
  _ConjunctiveFeature& operator=(_ConjunctiveFeature&&) = default;

  /// \brief Return the (sorted) index set of basis functions that make up this conjunctive feature
  const SizeVec getBaseFeatures() const {
      return _base_fns;
  }
  /// \brief Return unique hash of this conjunctive feature
  std::size_t getHash() const {
    return _hash;
  }

  /// \brief operators
  bool operator==(const _ConjunctiveFeature<T>& other) const {
    return _hash == other._hash;
  }
  bool operator!=(const _ConjunctiveFeature<T>& other) const {
    return !(*this == other);
  }
  /// \brief The number of conjunctions that make up this function
  const Size size() {
    return _base_fns.size();
  }
};
template<class T>
using ConjunctiveFeature = boost::shared_ptr<_ConjunctiveFeature<T>>;

template<class T>
const std::size_t _ConjunctiveFeature<T>::EMPTY_HASH = 0;

/// \brief Stream output of a \a ConjunctiveFeature
template<class T>
std::ostream& operator<<(std::ostream &os, const _ConjunctiveFeature<T>& f) {
  os << "Conj({";
  for(auto h : f.getBaseFeatures()) {
      os << "h" << h << "^";
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
  BasisScore(const Domain& domain, const FactoredValueFunction& vfn)
  : _domain(domain), _value_fn(vfn) { }
  /// \brief The (abstract) scoring function for the given basis function
  virtual double score(const _DiscreteFunction<Reward>* basis) const = 0;
  /// \brief The name of this scoring function
  virtual std::string getName() const = 0;
};

/**
 * \brief Computing (max BE(f) - min BE(f)) over the region where the feature f is active
 */
class EpsilonScore : public BasisScore {
public:
  /// \brief ctor
  /// \param vfn The (solved) factored value function that will be used for scoring
  EpsilonScore(const Domain& domain, const FactoredValueFunction& vfn)
  : BasisScore(domain, vfn) { }
  /// \brief score implementation
  double score(const _DiscreteFunction<Reward>* basis) const;
  std::string getName() const {
    return "EpsilonScore";
  }
};

/**
 * \brief Computing the BEBF score Sum_BE(f) / sqrt(|Coverage(f)|) over the region where feature f is active
 */
class BEBFScore : public BasisScore {
  /// \brief ctor
  /// \param vfn The (solved) factored value function that will be used for scoring
  BEBFScore(const Domain& domain, const FactoredValueFunction& vfn)
  : BasisScore(domain, vfn) { }
  /// \brief score implementation
  double score(const _DiscreteFunction<Reward>* basis) const;
  std::string getName() const {
    return "BEBFScore";
  }
};

/**
 * \brief A BasisGenerator uses a scoring mechanism to compute the next best basis
 * \tparam It The iterator to compute new candidate basis Id-tuples
 * \tparam Sc The type of the scoring method
 */
template<class It, class Sc>
class BasisGenerator {
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
//  std::unordered_set
public:
  BasisGenerator(const Domain& domain, FactoredValueFunction vfn, std::string name = "")
  : _domain(domain), _value_fn(std::move(vfn)), _score(_domain, _value_fn), _name(name) { }

  /// \brief Return the next best basis function
  DiscreteFunction<Reward> nextBest() const {
    // Note: sort by size?
    LOG_DEBUG("Basis size: " << _value_fn->getBasis().size());
    const auto& basisVec = _value_fn->getBasis();
    SizeVec basisIds = cpputil::ordered_vec<Size>(basisVec.size());
    It biter(basisIds);

    // TODO: check iterator for collisions (existing features)

    // Test:
    Size h1_id = 37;
    Size h2_id = 58;
    // hack to do a logical `AND' with BinOp over two functions
    int land = 0; // start with logical `OR' over first two values, then iterate `AND'
    DiscreteFunction<Reward> candf = algorithm::pair(h1_id, h2_id, basisVec[h1_id], basisVec[h2_id],
        [&land](Reward r1, Reward r2) -> Reward { return !(land++%2) ? r2 : (r1 != 0 && r2 != 0); });

    double s = _score.score(candf.get());
    LOG_DEBUG("<" << h1_id << "," << h2_id << ">: " << s);

    return candf;
  }
};

//
// Algorithms for basis function evaluation
//

namespace {

// helper function for algorithm::pair
template<class T, class It, class Ins>
inline void pair_helper(It it1b, It it1e, const _ConjunctiveFeature<T>* cf2, const Size (&h2_arr)[1], const Ins& ins) {
  if(cf2)
      std::set_union(it1b, it1e, std::begin(cf2->getBaseFeatures()), std::end(cf2->getBaseFeatures()), ins);
  else
      std::set_union(it1b, it1e, std::begin(h2_arr), std::end(h2_arr), ins);
}

} // anonymous ns

namespace algorithm {

/// \brief Compute the (flat, ordered) conjunction of original basis functions that make up joint feature
template<class T>
SizeVec pair_basis(Size h1_id, Size h2_id, const DiscreteFunction<T>& h1, const DiscreteFunction<T>& h2) {
  // compute basis features that are active in conjunction
  SizeVec joint_base;
  _ConjunctiveFeature<T>* cf1 = dynamic_cast<_ConjunctiveFeature<T>*>(h1.get());
  _ConjunctiveFeature<T>* cf2 = dynamic_cast<_ConjunctiveFeature<T>*>(h2.get());
  Size h1_arr[] = { h1_id };
  Size h2_arr[] = { h2_id };
  // do std::set_union in canonical form for all instances instead of using, e.g., boost::any_range
  auto ins = std::inserter(joint_base, joint_base.begin());
  if(cf1) {
      pair_helper(std::begin(cf1->getBaseFeatures()), std::end(cf1->getBaseFeatures()), cf2, h2_arr, ins);
  }
  else {
      pair_helper(std::begin(h1_arr), std::end(h1_arr), cf2, h2_arr, ins);
  }
  return joint_base;
}

/// \brief Compute the conjunction of two features defined over state factors
/// \param binOp The operation to perform when joining features (e.g., `AND')
/// \param joint_base The (flat, ordered) conjunction of original basis functions that make up joint feature
/// \see algorithm::join, algorithm::pair_basis
template<class T, class BinOp>
DiscreteFunction<T> pair(const SizeVec& joint_base, const DiscreteFunction<T>& h1, const DiscreteFunction<T>& h2, BinOp binOp) {
  assert(h1 && h2 && h1->getActionFactors().empty() && h2->getActionFactors().empty());
  // TODO: add optimization path for indicator functions
  // TODO: add optimization path when h1, h2 subdomains are identical/subsets, see _FDiscreteFunction::transform
  FDiscreteFunction<T> jf = boost::static_pointer_cast<_FDiscreteFunction<T>>(algorithm::join({h1,h2}, binOp));
  ConjunctiveFeature<T> cof = boost::make_shared<_ConjunctiveFeature<T>>(std::move(*jf));

  // update basis features that are active in conjunction
  auto& base_fns = cof->_base_fns;
  base_fns = joint_base;
  // update hash
  cof->_hash = boost::hash_range(base_fns.begin(), base_fns.end());
  return cof;
}

/// \brief Compute the conjunction (`AND') of two features defined over state factors
/// This is a specialization of the general algorithm::join to obtain a ConjunctiveFeature
/// \param h1_id The unique Id of function h1 (e.g., position in basis vector)
/// \param h2_id The unique Id of function h2 (e.g., position in basis vector)
/// \see algorithm::join
template<class T, class BinOp>
DiscreteFunction<T> pair(Size h1_id, Size h2_id, const DiscreteFunction<T>& h1, const DiscreteFunction<T>& h2, BinOp binOp) {
  // compute basis features that are active in conjunction
  SizeVec joint_base = pair_basis(h1_id, h2_id, h1, h2);
  return pair(joint_base, h1, h2, binOp);
}

/// \brief Perform a given operation (e.g., max, min, sum) on the \a FactoredFunction for all values where the basis is `active'
/// \note Assumes that the FactoredFunction is defined over the same domain as `basis'
template<class T, class BinOp>
T evalOpOverBasis(const _DiscreteFunction<T>* basis, const FactoredFunction<T>& facfn, bool known_flat, T init, BinOp binOp) {
  assert(basis->getActionFactors().empty());
  // optimization path for basis functions with tabular storage
  const _FDiscreteFunction<T>* of = is_flat(basis, known_flat);
  const std::vector<T>* pvals = nullptr;
  typename std::vector<T>::size_type j = 0;
  if(of) {
      pvals = &of->values();
  }

  // obtain value of facfn empty functionals
  T val = T(0);
  for(const auto& empty_fn : std::get<1>(facfn)) {
      val += empty_fn->eval(State(),Action());
  }

  // manually loop over basis function domain and apply binOp
  // Note: for variable elimination cases (min, max), I could continue to run that instead!
  const subdom_map s_map(basis->getStateFactors());
  T manVal = init;
  const _Indicator<T>* pI = dynamic_cast<const _Indicator<T>*>(basis);
  if(pI) {
      // basis is active in exactly one state
      const State s(basis->getSubdomain(), pI->getStateIndex());
      manVal = T(0);
      for(const auto& f : std::get<0>(facfn)) {
          State ms = f->mapState(s,s_map);
          manVal += f->eval(ms,Action());
      }
  }
  else {
      _StateIncrementIterator sitr(basis->getSubdomain());
      while(sitr.hasNext()) {
          const State& s = sitr.next();
          T basisVal = of ? (*pvals)[j++] : (*basis)(s,Action());
          if(basisVal != T(0)) { // basis is active in `s'
              T v = T(0); // candidate value
              for(const auto& f : std::get<0>(facfn)) {
                  State ms = f->mapState(s,s_map);
                  v += f->eval(ms,Action());
              }
              binOp(manVal, v);
          }
      }
  }
  val += manVal;
  return val;
}

} // namespace algorithm

} // namespace crl

#endif /*BASIS_GEN_HPP_*/
