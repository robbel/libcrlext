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
#include <numeric>
#include "crl/function.hpp"
#include "logger.hpp"

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

/// \brief Given a partial instantiation of either (local) \a State s or \a Action a, return the slice for (global) variable `i'
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
        // convert `i' to local scope index
        i = cpputil::inverse_map<Size>(pf->getStateFactors())(i);
    }
    else {
        prl = &ra;
        i -= num_state;
        range = pf->_domain->getActionRanges()[i];
        // convert `i' to local scope index
        i = cpputil::inverse_map<Size>(pf->getActionFactors())(i);
    }

    for(Factor fa = range.getMin(); fa <= range.getMax(); fa++) {
        prl->setFactor(i, fa);
        sl.push_back(pf->eval(rs,ra));
    }

    return sl;
}

/// \brief Maximize the given function over a particular state or action variable `i' (max marginalization)
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
  const Size num_actions = f->_subdomain->getNumActions();
  auto& vals = f->values();

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
      v = of ? (*pvals)[j++] : (*pf)(s,a);
      State ms = f->mapState(s,s_dom);
      Action ma = f->mapAction(a,a_dom);
      const auto idx = ms.getIndex()*num_actions+ma.getIndex();
      if(v > vals[idx]) {
          vals[idx] = v;
      }
  }

  return f;
}

/// \brief Marginalize the given function over a particular state or action variable `i' (i.e., `sum out' i)
/// \param known_flat True iff function `pf' is known to be a \a _FDiscreteFunction (optimization)
/// \return A new function that is marginalized over (and does not depend on) `i'
/// \note If `i' is greater than the number of state factors, it is assumed to be an action factor
template<class T>
DiscreteFunction<T> marginalize(const _DiscreteFunction<T>* pf, Size i, bool known_flat) {
  const _FDiscreteFunction<T>* of = is_flat(pf, known_flat);

  // TODO: optimization paths for Indicator and ConstantFn
  FDiscreteFunction<T> f = boost::make_shared<_FDiscreteFunction<T>>(pf->_domain);
  f->_state_dom = pf->getStateFactors();
  f->_action_dom = pf->getActionFactors();
  f->eraseFactor(i);
  f->pack();
  const Size num_actions = f->_subdomain->getNumActions();
  auto& vals = f->values();

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
      // determine sum
      T v = 0;
      v = of ? (*pvals)[j++] : (*pf)(s,a);
      State ms = f->mapState(s,s_dom);
      Action ma = f->mapAction(a,a_dom);
      const auto idx = ms.getIndex()*num_actions+ma.getIndex();
      vals[idx] += v;
  }

  return f;
}

/// \brief Joins the given set of functions into a larger one defined as the function sum over the joint domain
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

//
// Variable elimination with argmax
//

/// \brief Runs variable elimination on the provided \a FunctionSet (the forward pass).
/// The function set itself is modified in the process.
/// \return A tuple of (0) the generated intermediate functions, and (1) the generated functions with empty scope
template<class T>
std::tuple<std::vector<DiscreteFunction<T>>, std::vector<DiscreteFunction<T>>>
variableElimination(FunctionSet<T>& F, const crl::SizeVec& elimination_order) {
  LOG_DEBUG("Number of variables to eliminate: " << elimination_order.size() << " out of " << F.getNumFactors());

  std::vector<DiscreteFunction<T>> elim_cache;
  // store for functions that have reached empty scope
  std::vector<DiscreteFunction<T>> empty_fns;
  using range = typename FunctionSet<T>::range;
  for(Size v : elimination_order) {
      LOG_DEBUG("Eliminating variable " << v);
      range r = F.getFactor(v);
      if(!r.hasNext()) {
          LOG_INFO("Variable " << v << " not eliminated. It does not exist in FunctionSet.");
          continue;
      }

      DiscreteFunction<T> esum = algorithm::join(r);
      DiscreteFunction<T> emax = algorithm::maximize(esum.get(), v, false);
      elim_cache.push_back(std::move(esum));
      F.eraseFactor(v);
      if(emax->getSubdomain()->getNumStateFactors() == 0 && emax->getSubdomain()->getNumActionFactors() == 0) {
        LOG_DEBUG("Function reduced to empty scope");
        empty_fns.push_back(std::move(emax));
      }
      else {
        F.insert(std::move(emax)); // continue elimination
      }
  }

  return std::make_tuple(elim_cache,empty_fns);
}

/// \brief Runs variable elimination on the provided \a FunctionSet and computes the argmax (in a backward pass)
/// \return The maximizing variable setting and the (maximal) value
template<class T>
std::tuple<Action,T> argVariableElimination(FunctionSet<T>& F, const crl::SizeVec& elimination_order) {
  // Make sure we can eliminate all variables in function set
  assert(elimination_order.size() >= F.getNumFactors());
  // run variable elimination
  auto retFns = variableElimination(F, elimination_order);
  const std::vector<DiscreteFunction<T>>& elim_cache = std::get<0>(retFns);
  const std::vector<DiscreteFunction<T>>& empty_fns  = std::get<1>(retFns);

  if(!F.empty()) {
      throw cpputil::InvalidException("Functions remain in function set: argVariableElimination failed");
  }

  // sum over all empty functions defines the maximum value
  T maxVal = std::accumulate(empty_fns.begin(), empty_fns.end(), T(0), [](T store, const DiscreteFunction<T>& f) {
      return store + f->eval(State(),Action()); });

  if(elim_cache.empty()) {
      return std::make_tuple(Action(),maxVal);
  }

  // argmax in reverse order
  assert(elim_cache.size() == elimination_order.size());
  const Domain& domain = elim_cache.front()->_domain;
  const RangeVec ranges = domain->getActionRanges();
  const Size num_state = domain->getNumStateFactors();

  Action retMax(domain); // the globally maximizing joint action
  auto fit = elim_cache.rbegin(); // function iterator
  for (auto rit = elimination_order.rbegin(); rit != elimination_order.rend(); ++rit, ++fit) {
      Size v = *rit;
      assert(v >= num_state); // eliminating an action variable
      Action ma((*fit)->mapAction(retMax)); // map to local function's domain
      const std::vector<T>& slice = algorithm::slice(fit->get(), v, State(), ma);
      auto optIt = std::max_element(slice.begin(), slice.end()); // yields the /first/ maximizing action
      assert(optIt != slice.end()); // empty range check
      auto offset = optIt-slice.begin();
      // update maximizing action
      retMax.setFactor(v-num_state, ranges[v-num_state].getMin()+offset);
  }

  return std::make_tuple(retMax, maxVal);
}

} // namespace algorithm

} // namespace crl

#endif /*ALGORITHM_HPP_*/
