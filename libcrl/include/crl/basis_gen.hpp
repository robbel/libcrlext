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

#include "crl/alp.hpp"

namespace crl {

namespace algorithm {

/// \brief Score a basis function under different criteria (here: max Bellman error covered by `basis')
double scoreBasis(const Domain& domain, const DiscreteFunction<Reward>& basis, const FactoredValueFunction& fval);

/// \brief Perform a given operation (e.g., max, min, sum) on the \a FactoredFunction for all values where the basis is `active'
/// \note Assumes that the FactoredFunction is defined over the same domain as `basis'
template<class T, class BinOp>
T evalOpBasis(const _DiscreteFunction<T>* basis, const FactoredFunction<T>& facfn, bool known_flat, T init, BinOp binOp) {
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
  T manVal = init;
  const _Indicator<T>* pI = dynamic_cast<const _Indicator<T>*>(basis);
  if(pI) {
      // basis is active in exactly one state
      const State s(basis->getSubdomain(), pI->getStateIndex());
      manVal = T(0);
      for(const auto& f : std::get<0>(facfn)) {
          manVal += f->eval(s,Action());
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
                  v += f->eval(s,Action());
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
