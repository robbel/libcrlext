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

namespace crl {

namespace algorithm {

/// \brief Sum the given function over its entire domain
/// \param known_flat True iff function `pf' is known to be a \a _FDiscreteFunction (optimization)
template<class T>
T sum_over_domain(const _DiscreteFunction<T>* pf, bool known_flat) {
    const _FDiscreteFunction<T>* ff;
    if(known_flat) {
        ff = static_cast<const _FDiscreteFunction<T>*>(pf);
    } else {
        ff = dynamic_cast<const _FDiscreteFunction<T>*>(pf);
        if(ff) {
            known_flat = true;
        }
    }

    if(known_flat) {
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

/// \brief Instantiate the given function in a particular state
/// \param known_flat True iff function `pf' is known to be a \a _FDiscreteFunction (optimization)
/// \return A new function with only \a Action dependencies
template<class T>
DiscreteFunction<T> instantiate(const _DiscreteFunction<T>* pf, const State& s, bool known_flat) {
  const _FDiscreteFunction<T>* of;
  if(known_flat) {
      of = static_cast<const _FDiscreteFunction<T>*>(pf);
  } else {
      of = dynamic_cast<const _FDiscreteFunction<T>*>(pf);
      if(of) {
          known_flat = true;
      }
  }

  // TODO: additional optimization paths for Indicator and ConstantFn
  DiscreteFunction<T> f = boost::make_shared<_FDiscreteFunction<T>>(pf->_domain);
  _FDiscreteFunction<T>* rawf = static_cast<_FDiscreteFunction<T>*>(f.get());
  rawf->_action_dom = pf->getActionFactors();
  rawf->pack();
  const Size num_actions = rawf->_subdomain->getNumActions();

  if(known_flat) {
    auto& vals  = of->_sa_table->values();
    auto& start = vals[s.getIndex()*num_actions];
    auto start_it = vals.begin() + (&start - vals.data());
    std::copy(start_it, start_it+num_actions, rawf->values().begin());
  }
  else { // evaluate other function exhaustively
    _ActionIncrementIterator aitr(rawf->getSubdomain());
    auto& vals = rawf->values();
    typename std::vector<T>::size_type i = 0;
    while(aitr.hasNext()) {
      vals[i++] = pf->eval(s,aitr.next());
    }
  }

  return f;
}

/// \brief Maximize the given function over a particular state or action variable `i'
/// \param known_flat True iff function `pf' is known to be a \a _FDiscreteFunction (optimization)
/// \return A new function that is maximized over (and does not depend on) `i'
/// \note If `i' is greater than the number of state factors, it is assumed to be an action factor
template<class T>
DiscreteFunction<T> maximize(const _DiscreteFunction<T>* pf, Size i, bool known_flat) {
  const _FDiscreteFunction<T>* of;
  if(known_flat) {
      of = static_cast<const _FDiscreteFunction<T>*>(pf);
  } else {
      of = dynamic_cast<const _FDiscreteFunction<T>*>(pf);
      if(of) {
          known_flat = true;
      }
  }
#if 0
  if(known_flat) {
      auto& vals = this->values();
      for(T& v : vals) {
          const std::tuple<State,Action>& sa = saitr.next();
          v = 0.;
          hitr.reset();
          while(hitr.hasNext()) {
              const State& s = hitr.next();
              v += h[(Size)s] * _dbn.T(std::get<0>(sa), std::get<1>(sa), s, s_dom, h_dom, a_dom);
          }
      }

    }
#endif



  // TODO: optimization paths for Indicator and ConstantFn
  DiscreteFunction<T> f = boost::make_shared<_FDiscreteFunction<T>>(pf->_domain);
  const Size a_offset = f->_domain->getNumStateFactors();
  f->_state_dom = pf->getStateFactors();
  f->_action_dom = pf->getActionFactors();
  if(i < a_offset) {
    f->eraseStateFactor(i);
  }
  else {
    f->eraseActionFactor(i-a_offset);
  }

  _FDiscreteFunction<T>* rawf = static_cast<_FDiscreteFunction<T>*>(f.get());
  rawf->pack(-std::numeric_limits<T>::infinity());
#if 0
  _StateActionIncrementIterator saitr(pf->getSubdomain());
  while(saitr.hasNext()) {
      const std::tuple<State,Action>& sa = saitr.next();
      const State& s = std::get<0>(sa);
      const Action& a = std::get<1>(sa);
      T val =
      // determine max over old function
      State ms = f->mapState(s);
      Action ma = f->mapAction(a);
      if(*(f)(ms,ma) > )



  }
#endif
  //_FDiscreteFunction<T>* rawf = static_cast<_FDiscreteFunction<T>*>(f.get());
  //rawf->_action_dom = pf->_action_dom;
  //rawf->pack();


#if 0
  _FDiscreteFunction<T>* rawf = static_cast<_FDiscreteFunction<T>*>(f.get());
  rawf->_action_dom = pf->_action_dom;
  rawf->pack();
  const Size num_actions = rawf->_subdomain->getNumActions();

  if(known_flat) {
    auto& vals  = of->_sa_table->values();
    auto& start = vals[s.getIndex()*num_actions];
    auto start_it = vals.begin() + (&start - vals.data());
    std::copy(start_it, start_it+num_actions, rawf->values().begin());
  }
  else { // evaluate other function exhaustively
    _ActionIncrementIterator aitr(rawf->getSubdomain());
    auto& vals = rawf->values();
    typename std::vector<T>::size_type i = 0;
    while(aitr.hasNext()) {
      vals[i++] = pf->eval(s,aitr.next());
    }
  }
#endif
  return f;
}

} // namespace algorithm

} // namespace crl

#endif /*ALGORITHM_HPP_*/
