/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#ifndef ALGORITHM_HPP_
#define ALGORITHM_HPP_

#include <iostream>
#include "crl/function.hpp"

//
// Some useful algorithms on functions
//

namespace {

using namespace crl;

/// \brief Returns a valid pointer if passed function has a tabular representation, nullptr otherwise
template<class T>
const _FDiscreteFunction<T>* is_flat(const _DiscreteFunction<T>* pf, bool known_flat) {
    return(known_flat ? static_cast<const _FDiscreteFunction<T>*>(pf) : dynamic_cast<const _FDiscreteFunction<T>*>(pf));
}

} // anonymous ns

namespace crl {

namespace algorithm {

/// \brief Sum the given function over its entire domain
/// \param known_flat True iff function `pf' is known to be a \a _FDiscreteFunction (optimization)
template<class T>
T sum_over_domain(const _DiscreteFunction<T>* pf, bool known_flat) {
    const _FDiscreteFunction<T>* ff = is_flat(pf, known_flat);

    if(ff) {
        return std::accumulate(ff->_sa_table->values().begin(), ff->_sa_table->values().end(), T(0));
    }
    else { // some other cases FIXME move domainSum into base class
        const _Indicator<T>* pif = dynamic_cast<const _Indicator<T>*>(pf);
        if(pif) {
            return 1;
        } else { // brute force summation over domain
            T sum = 0;
            _StateActionIncrementIterator saitr(pf->getSubdomain());
            while(saitr.hasNext()) {
                const std::tuple<State,Action>& sa = saitr.next();
                sum += pf->eval(std::get<0>(sa), std::get<1>(sa));
            }
            return sum;
        }
    }
}

/// \brief Instantiate the given function in a particular (complete w.r.t. function's subdomain) state
/// \param known_flat True iff function `pf' is known to be a \a _FDiscreteFunction (optimization)
/// \note This is a less general version of adding evidence (over some subset of variables) to the function pf
/// \return A new function with only \a Action dependencies
template<class T>
DiscreteFunction<T> instantiate(const _DiscreteFunction<T>* pf, const State& s, bool known_flat) {
  const _FDiscreteFunction<T>* of = is_flat(pf, known_flat);

  // TODO: additional optimization paths for Indicator and ConstantFn
  FDiscreteFunction<T> f = boost::make_shared<_FDiscreteFunction<T>>(pf->_domain);
  f->_action_dom = pf->getActionFactors();
  f->pack();

  if(of) {
    const Size num_actions = f->_subdomain->getNumActions();
    auto& vals  = of->_sa_table->values();
    auto& start = vals[s.getIndex()*num_actions];
    auto start_it = vals.begin() + (&start - vals.data());
    std::copy(start_it, start_it+num_actions, f->values().begin());
  }
  else { // evaluate other function exhaustively
    _ActionIncrementIterator aitr(f->getSubdomain());
    auto& vals = f->values();
    typename std::vector<T>::size_type i = 0;
    while(aitr.hasNext()) {
      vals[i++] = pf->eval(s,aitr.next());
    }
  }

  return f;
}

/// \brief Given a partial instantiation of either \a State s or \a Action a, return the slice for variable `i'
/// \note This is a less general version of adding evidence (over some subset of variables) to the function pf
/// \note If `i' is greater than the number of state factors, it is assumed to be an action factor
/// \return A slice with only variable `i' dependencies
template<class T>
std::vector<T> slice(const _DiscreteFunction<T>* pf, Size i, const State&s, const Action& a) {
    std::vector<T> sl;
    const Size num_state = pf->_domain->getNumStateFactors();
    FactorRange range;
    // select relevant free variable
    State rs(s);
    Action ra(a);
    RLType* prl = nullptr;
    if(i < num_state) {
        prl = &rs;
        range = pf->_domain->getStateRanges()[i];
    }
    else {
        prl = &ra;
        i -= num_state;
        range = pf->_domain->getActionRanges()[i];
    }

    for(Factor fa = range.getMin(); fa <= range.getMax(); fa++) {
        prl->setFactor(i, fa);
        sl.push_back(pf->eval(rs,ra));
    }

    return sl;
}

/// \brief Maximize the given function over a particular state or action variable `i'
/// \param known_flat True iff function `pf' is known to be a \a _FDiscreteFunction (optimization)
/// \return A new function that is maximized over (and does not depend on) `i'
/// \note If `i' is greater than the number of state factors, it is assumed to be an action factor
template<class T>
DiscreteFunction<T> maximize(const _DiscreteFunction<T>* pf, Size i, bool known_flat) {
  const _FDiscreteFunction<T>* of = is_flat(pf, known_flat);

  // TODO: optimization paths for Indicator and ConstantFn
  FDiscreteFunction<T> f = boost::make_shared<_FDiscreteFunction<T>>(pf->_domain);
  f->_state_dom = pf->getStateFactors();
  f->_action_dom = pf->getActionFactors();
  f->eraseFactor(i);
  f->pack(-std::numeric_limits<T>::infinity());

  _StateActionIncrementIterator saitr(pf->getSubdomain());
  const subdom_map s_dom(pf->getStateFactors());
  const subdom_map a_dom(pf->getActionFactors());
  const std::vector<T>* pvals = nullptr;
  typename std::vector<T>::size_type j = 0;
  if(of) {
      pvals = &of->values();
  }

  while(saitr.hasNext()) {
      const std::tuple<State,Action>& sa = saitr.next();
      const State& s = std::get<0>(sa);
      const Action& a = std::get<1>(sa);
      // determine max over old function
      T v = 0;
      if(of)
          v = (*pvals)[j++];
      else
          v = (*pf)(s,a);
      State ms = f->mapState(s,s_dom);
      Action ma = f->mapAction(a,a_dom);
      if(v > (*f)(ms,ma)) {
        f->define(ms,ma,v);
      }
  }

  return f;
}

/// \brief Joins the given set of functions into a larger one defined over the joint domain
/// \param f An operator to apply when joining two successive functions from `funcs' together
/// \return A new function defined over the union of all function domains represented in `funcs'
template<class T>
DiscreteFunction<T> join(cpputil::Iterator<DiscreteFunction<T>>& funcs) {
    assert(funcs.hasNext());
    const Domain& domain = funcs.next()->_domain; // same domain
    FDiscreteFunction<T> jf = boost::make_shared<_FDiscreteFunction<T>>(domain);
    // join function scopes
    funcs.reset();
    while(funcs.hasNext()) {
        jf->join(*funcs.next());
    }
    // allocate memory
    jf->pack();
    // create joint tabular values
    _StateActionIncrementIterator saitr(jf->getSubdomain());
    const subdom_map s_dom(jf->getStateFactors());
    const subdom_map a_dom(jf->getActionFactors());
    auto& vals = jf->values();
    typename std::vector<T>::size_type i = 0;
    while(saitr.hasNext()) {
        const std::tuple<State,Action>& sa = saitr.next();
        const State& s = std::get<0>(sa);
        const Action& a = std::get<1>(sa);
        T v = 0;
        funcs.reset();
        while(funcs.hasNext()) {
            const auto& f = funcs.next();
            State ms = f->mapState(s,s_dom);
            Action ma = f->mapAction(a,a_dom);
            v += f->eval(ms,ma);
        }
        vals[i++] = v;
    }

    return jf;
}

/// \brief Convenience function
template<class T>
DiscreteFunction<T> join(std::initializer_list<DiscreteFunction<T>> funcs) {
    cpputil::ContainerIterator<DiscreteFunction<double>,std::initializer_list<DiscreteFunction<double>>> fiter(std::move(funcs));
    return join(fiter);
}

} // namespace algorithm

} // namespace crl

#endif /*ALGORITHM_HPP_*/
